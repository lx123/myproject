
/**
 * @file uploader.cpp
 * Firmware uploader for PX4IO
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <sys/stat.h>
#include <nuttx/arch.h>

#include <crc32.h>

#include "uploader.h"

#include <board_config.h>

// define for comms logging
//#define UDEBUG

PX4IO_Uploader::PX4IO_Uploader() :  //构造函数
	_io_fd(-1),   //io文件句柄
	_fw_fd(-1),   //固件句柄
	bl_rev(0)     //bootloader版本
{
}

PX4IO_Uploader::~PX4IO_Uploader()
{
}

int
PX4IO_Uploader::upload(const char *filenames[])  //传递过来文件
{
	int	ret;
	const char *filename = NULL;
	size_t fw_size;

#ifndef PX4IO_SERIAL_DEVICE
#error Must define PX4IO_SERIAL_DEVICE in board configuration to support firmware upload
#endif

	/* allow an early abort and look for file first */  //允许一个早起的终止，先寻找文件
	for (unsigned i = 0; filenames[i] != nullptr; i++) {  //对文件名进行扫描
		_fw_fd = open(filenames[i], O_RDONLY);  //只读打开文件

		if (_fw_fd < 0) {  //文件打开失败
			log("failed to open %s", filenames[i]);
			continue;
		}

		log("using firmware from %s", filenames[i]); //文件打开成功
		filename = filenames[i];  //转存文件名称
		break;
	}

	if (filename == NULL) {  //说明文件没有找到
		log("no firmware found");
		close(_io_fd);  //关闭io设备
		_io_fd = -1;
		return -ENOENT;
	}

	_io_fd = open(PX4IO_SERIAL_DEVICE, O_RDWR);  //打开串口设备

	if (_io_fd < 0) {  //文件打开失败
		log("could not open interface");
		return -errno;
	}

	/* save initial uart configuration to reset after the update */  //保存初始串口配置，以便在更新后复位
	struct termios t_original;
	tcgetattr(_io_fd, &t_original);

	/* adjust line speed to match bootloader */   //自适应行速匹配bootloader
	struct termios t;
	tcgetattr(_io_fd, &t);
	cfsetspeed(&t, 115200);  //设置串口速度
	tcsetattr(_io_fd, TCSANOW, &t);  //立即改变串口属性

	/* look for the bootloader for 150 ms */  //寻找bootloader
	for (int i = 0; i < 15; i++) {  //与io板里的bootloader取得同步
		ret = sync();

		if (ret == OK) {  //如果取得成功
			break;

		} else {
			usleep(10000); //休息10ms
		}
	}

	if (ret != OK) {  //同步失败
		/* this is immediately fatal */  //这样出现致命错误
		log("bootloader not responding");
		tcsetattr(_io_fd, TCSANOW, &t_original); //复原串口属性
		close(_io_fd);  //关闭io板设备
		_io_fd = -1;
		return -EIO;
	}

	struct stat st;

	if (stat(filename, &st) != 0) {  //获取文件属性
		log("Failed to stat %s - %d\n", filename, (int)errno);
		tcsetattr(_io_fd, TCSANOW, &t_original);  //还原串口
		close(_io_fd);  //关闭io设备
		_io_fd = -1;
		return -errno;
	}

	fw_size = st.st_size;  //固件大小

	if (_fw_fd == -1) {  //固件句柄
		tcsetattr(_io_fd, TCSANOW, &t_original);  //还原串口属性
		close(_io_fd);
		_io_fd = -1;
		return -ENOENT;
	}

	/* do the usual program thing - allow for failure */  //执行通用变成事件，允许失败
	for (unsigned retries = 0; retries < 1; retries++) {  //尝试次数，只执行一次
		if (retries > 0) { //如果尝试次数大于1
			log("retrying update...");
			ret = sync();

			if (ret != OK) {
				/* this is immediately fatal */
				log("bootloader not responding");
				tcsetattr(_io_fd, TCSANOW, &t_original);
				close(_io_fd);
				_io_fd = -1;
				return -EIO;
			}
		}

		ret = get_info(INFO_BL_REV, bl_rev);  //获取io控制板bootloader的版本

		if (ret == OK) {  //获取成功
			if (bl_rev <= BL_REV) { //兼容以前的版本
				log("found bootloader revision: %d", bl_rev);

			} else {
				log("found unsupported bootloader revision %d, exiting", bl_rev);  //bootloader版本不支持
				tcsetattr(_io_fd, TCSANOW, &t_original);
				close(_io_fd);
				_io_fd = -1;
				return OK;
			}
		}

		ret = erase();  //擦除flash

		if (ret != OK) {  //判断察除是否成功
			log("erase failed");
			continue;
		}

		ret = program(fw_size);  //编程，给出文件大小

		if (ret != OK) {
			log("program failed");
			continue;
		}

		if (bl_rev <= 2) {  //如果版本号小于2
			ret = verify_rev2(fw_size);  //校验

		} else {
			/* verify rev 3 and higher. Every version *needs* to be verified. */
			ret = verify_rev3(fw_size);
		}

		if (ret != OK) {  //校验失败
			log("verify failed");
			continue;
		}

		ret = reboot();  //io板复位

		if (ret != OK) {  //复位成功
			log("reboot failed");
			tcsetattr(_io_fd, TCSANOW, &t_original);  //恢复串口属性
			close(_io_fd);
			_io_fd = -1;
			return ret;
		}

		log("update complete");  //固件更新成功

		ret = OK;
		break;
	}

	/* reset uart to previous/default baudrate */
	tcsetattr(_io_fd, TCSANOW, &t_original);  //复位串口到之前的波特率

	close(_fw_fd);  //关闭固件句柄
	close(_io_fd);  //关闭io句柄
	_io_fd = -1;

	// sleep for enough time for the IO chip to boot. This makes
	// forceupdate more reliably startup IO again after update
	up_udelay(100 * 1000); //等待io板复位

	return ret;
}

int
PX4IO_Uploader::recv_byte_with_timeout(uint8_t *c, unsigned timeout)  //接收数据
{
	struct pollfd fds[1]; //轮询结构体

	fds[0].fd = _io_fd;
	fds[0].events = POLLIN;

	/* wait <timout> ms for a character */ //等待超时事件的一个字符
	int ret = ::poll(&fds[0], 1, timeout);  //轮询

	if (ret < 1) {
#ifdef UDEBUG
		log("poll timeout %d", ret);
#endif
		return -ETIMEDOUT;
	}

	read(_io_fd, c, 1); //读取一个字符
#ifdef UDEBUG
	log("recv_bytes 0x%02x", c);
#endif
	return OK;
}

int
PX4IO_Uploader::recv_bytes(uint8_t *p, unsigned count)  //接收字节，指定长度
{
	int ret = OK;

	while (count--) {  //读取指定数量的字节
		ret = recv_byte_with_timeout(p++, 5000);

		if (ret != OK) {
			break;
		}
	}

	return ret;
}

void
PX4IO_Uploader::drain()  //
{
	uint8_t c;
	int ret;

	do {
		// the small recv_bytes timeout here is to allow for fast
		// drain when rebooting the io board for a forced
		// update of the fw without using the safety switch
		ret = recv_byte_with_timeout(&c, 40);

#ifdef UDEBUG

		if (ret == OK) {
			log("discard 0x%02x", c);
		}

#endif
	} while (ret == OK);
}

int
PX4IO_Uploader::send(uint8_t c)  //发送
{
#ifdef UDEBUG
	log("send 0x%02x", c);
#endif

	if (write(_io_fd, &c, 1) != 1) {  //写入一个字节
		return -errno;
	}

	return OK;
}

int
PX4IO_Uploader::send(uint8_t *p, unsigned count)  //函数重载，指定长度字符
{
	int ret;

	while (count--) {
		ret = send(*p++);

		if (ret != OK) {
			break;
		}
	}

	return ret;
}

int
PX4IO_Uploader::get_sync(unsigned timeout)  //获取同步
{
	uint8_t c[2];
	int ret;

	ret = recv_byte_with_timeout(c, timeout);  //接收一个字节

	if (ret != OK) {
		return ret;
	}

	ret = recv_byte_with_timeout(c + 1, timeout);

	if (ret != OK) {
		return ret;
	}

	if ((c[0] != PROTO_INSYNC) || (c[1] != PROTO_OK)) {
		log("bad sync 0x%02x,0x%02x", c[0], c[1]);
		return -EIO;
	}

	return OK;
}

int
PX4IO_Uploader::sync()  //同步
{
	drain();

	/* complete any pending program operation */  //完成任何一个挂起的编程操作
	for (unsigned i = 0; i < (PROG_MULTI_MAX + 6); i++) {
		send(0);
	}

	send(PROTO_GET_SYNC); //发送获取同步
	send(PROTO_EOC);
	return get_sync();  //获取同步
}

int
PX4IO_Uploader::get_info(int param, uint32_t &val)  //获取信息
{
	int ret;

	send(PROTO_GET_DEVICE); //获取设备
	send(param);
	send(PROTO_EOC);

	ret = recv_bytes((uint8_t *)&val, sizeof(val)); //接收指定字节

	if (ret != OK) {
		return ret;
	}

	return get_sync(); //获取同步
}

int
PX4IO_Uploader::erase()  //察除flash
{
	log("erase...");
	send(PROTO_CHIP_ERASE); //发送察除指令
	send(PROTO_EOC);
	return get_sync(10000);		/* allow 10s timeout */
}


static int read_with_retry(int fd, void *buf, size_t n) //重新尝试读取
{
	int ret;
	uint8_t retries = 0;

	do {
		ret = read(fd, buf, n);
	} while (ret == -1 && retries++ < 100);

	if (retries != 0) {
		printf("read of %u bytes needed %u retries\n",
		       (unsigned)n,
		       (unsigned)retries);
	}

	return ret;
}

int
PX4IO_Uploader::program(size_t fw_size)  //编程
{
	uint8_t	*file_buf;
	ssize_t count;
	int ret;
	size_t sent = 0;

	file_buf = new uint8_t[PROG_MULTI_MAX];  //创建空间

	if (!file_buf) {
		log("Can't allocate program buffer");
		return -ENOMEM;
	}

	ASSERT((fw_size & 3) == 0);
	ASSERT((PROG_MULTI_MAX & 3) == 0);

	log("programming %u bytes...", (unsigned)fw_size);  //显示编程字节数

	ret = lseek(_fw_fd, 0, SEEK_SET);

	while (sent < fw_size) { //如果发送的字节小于固件的字节
		/* get more bytes to program */
		size_t n = fw_size - sent;

		if (n > PROG_MULTI_MAX) {
			n = PROG_MULTI_MAX;
		}

		count = read_with_retry(_fw_fd, file_buf, n);

		if (count != (ssize_t)n) {
			log("firmware read of %u bytes at %u failed -> %d errno %d",
			    (unsigned)n,
			    (unsigned)sent,
			    (int)count,
			    (int)errno);
			ret = -errno;
			break;
		}

		sent += count;

		send(PROTO_PROG_MULTI);
		send(count);
		send(file_buf, count);
		send(PROTO_EOC);

		ret = get_sync(1000);

		if (ret != OK) {
			break;
		}
	}

	delete [] file_buf;
	return ret;
}

int
PX4IO_Uploader::verify_rev2(size_t fw_size)  //校验版本2
{
	uint8_t	file_buf[4];
	ssize_t count;
	int ret;
	size_t sent = 0;

	log("verify...");
	lseek(_fw_fd, 0, SEEK_SET);

	send(PROTO_CHIP_VERIFY); //发送芯片检查
	send(PROTO_EOC);
	ret = get_sync();

	if (ret != OK) {
		return ret;
	}

	while (sent < fw_size) {
		/* get more bytes to verify */
		size_t n = fw_size - sent;

		if (n > sizeof(file_buf)) {
			n = sizeof(file_buf);
		}

		count = read_with_retry(_fw_fd, file_buf, n);

		if (count != (ssize_t)n) {
			log("firmware read of %u bytes at %u failed -> %d errno %d",
			    (unsigned)n,
			    (unsigned)sent,
			    (int)count,
			    (int)errno);
		}

		if (count == 0) {
			break;
		}

		sent += count;

		if (count < 0) {
			return -errno;
		}

		ASSERT((count % 4) == 0);

		send(PROTO_READ_MULTI);
		send(count);
		send(PROTO_EOC);

		for (ssize_t i = 0; i < count; i++) {
			uint8_t c;

			ret = recv_byte_with_timeout(&c, 5000);

			if (ret != OK) {
				log("%d: got %d waiting for bytes", sent + i, ret);
				return ret;
			}

			if (c != file_buf[i]) {
				log("%d: got 0x%02x expected 0x%02x", sent + i, c, file_buf[i]);
				return -EINVAL;
			}
		}

		ret = get_sync();

		if (ret != OK) {
			log("timeout waiting for post-verify sync");
			return ret;
		}
	}

	return OK;
}

int
PX4IO_Uploader::verify_rev3(size_t fw_size_local)  //校验版本3
{
	int ret;
	uint8_t	file_buf[4];
	ssize_t count;
	uint32_t sum = 0;
	uint32_t bytes_read = 0;
	uint32_t crc = 0;
	uint32_t fw_size_remote;
	uint8_t fill_blank = 0xff;

	log("verify...");
	lseek(_fw_fd, 0, SEEK_SET);

	ret = get_info(INFO_FLASH_SIZE, fw_size_remote);
	send(PROTO_EOC);

	if (ret != OK) {
		log("could not read firmware size");
		return ret;
	}

	/* read through the firmware file again and calculate the checksum*/
	while (bytes_read < fw_size_local) {
		size_t n = fw_size_local - bytes_read;

		if (n > sizeof(file_buf)) {
			n = sizeof(file_buf);
		}

		count = read_with_retry(_fw_fd, file_buf, n);

		if (count != (ssize_t)n) {
			log("firmware read of %u bytes at %u failed -> %d errno %d",
			    (unsigned)n,
			    (unsigned)bytes_read,
			    (int)count,
			    (int)errno);
		}

		/* set the rest to ff */
		if (count == 0) {
			break;
		}

		/* stop if the file cannot be read */
		if (count < 0) {
			return -errno;
		}

		/* calculate crc32 sum */
		sum = crc32part((uint8_t *)&file_buf, sizeof(file_buf), sum);

		bytes_read += count;
	}

	/* fill the rest with 0xff */
	while (bytes_read < fw_size_remote) {
		sum = crc32part(&fill_blank, sizeof(fill_blank), sum);
		bytes_read += sizeof(fill_blank);
	}

	/* request CRC from IO */
	send(PROTO_GET_CRC);
	send(PROTO_EOC);

	ret = recv_bytes((uint8_t *)(&crc), sizeof(crc));

	if (ret != OK) {
		log("did not receive CRC checksum");
		return ret;
	}

	/* compare the CRC sum from the IO with the one calculated */
	if (sum != crc) {
		log("CRC wrong: received: %d, expected: %d", crc, sum);
		return -EINVAL;
	}

	return OK;
}

int
PX4IO_Uploader::reboot()  //复位
{
	send(PROTO_REBOOT);  //发送复位指令
	up_udelay(100 * 1000); // Ensure the farend is in wait for char.
	send(PROTO_EOC);

	return OK;
}

void
PX4IO_Uploader::log(const char *fmt, ...)  //日志
{
	va_list	ap;

	printf("[PX4IO] ");
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	printf("\n");
	fflush(stdout);
}
