/**
 * @file i2c.cpp
 *
 * Base class for devices attached via the I2C bus.
 *
 * @todo Bus frequency changes; currently we do nothing with the value
 *       that is supplied.  Should we just depend on the bus knowing?
 */

#include "i2c_nuttx.h"

namespace device
{
unsigned int I2C::_bus_clocks[3] = { 100000, 100000, 100000 }; //总线时钟100k

I2C::I2C(const char *name,
	 const char *devname,
	 int bus,
	 uint16_t address,
	 uint32_t frequency,
	 int irq) :
	// base class
	CDev(name, devname, irq), //注册字符设备
	// public
	// protected
	_retries(0), //错误尝试次数
	// private
	_bus(bus), //总线号
	_address(address), //地址
	_frequency(frequency), //频率
	_dev(nullptr)
{
	// fill in _device_id fields for a I2C device
	_device_id.devid_s.bus_type = DeviceBusType_I2C; //总线类型
	_device_id.devid_s.bus = bus; //总线号
	_device_id.devid_s.address = address;  //设备地址
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;  //设备类型

//	_config.frequency = frequency;
//	_config.address = address;
//	_config.addrlen = 8;
}

I2C::~I2C()
{
	if (_dev) {
		stm32_i2cbus_uninitialize(_dev);  //注销设备
		_dev = nullptr;
	}
}


int
I2C::set_bus_clock(unsigned bus, unsigned clock_hz) //设置总线时钟
{
	int index = bus - 1;

	if (index < 0 || index >= static_cast<int>(sizeof(_bus_clocks) / sizeof(_bus_clocks[0]))) {  //判断i2c是否正确
		return -EINVAL;
	}

	if (_bus_clocks[index] > 0) {
		// DEVICE_DEBUG("overriding clock of %u with %u Hz\n", _bus_clocks[index], clock_hz);
	}

	_bus_clocks[index] = clock_hz; //设定特定时钟

	return OK;
}


int
I2C::init()
{
	int ret = OK;
	unsigned bus_index;

	// attach to the i2c bus
    _dev = stm32_i2cbus_initialize(_bus); //系统函数，总线初始化
    i2c_register(_dev , _bus);

	if (_dev == nullptr) { //初始化失败
		DEVICE_DEBUG("failed to init I2C");
		ret = -ENOENT;
		goto out;
	}

	// the above call fails for a non-existing bus index,
	// so the index math here is safe.
	bus_index = _bus - 1;

	// abort if the max frequency we allow (the frequency we ask)
	// is smaller than the bus frequency
	if (_bus_clocks[bus_index] > _frequency) {
		(void)stm32_i2cbus_uninitialize(_dev);
		_dev = nullptr;
		DEVICE_LOG("FAIL: too slow for bus #%u: %u KHz, device max: %u KHz)",
			   _bus, _bus_clocks[bus_index] / 1000, _frequency / 1000);
		ret = -EINVAL;
		goto out;
	}

	// set the bus frequency on the first access if it has
	// not been set yet
	if (_bus_clocks[bus_index] == 0) {  //如果总线时钟没有设置
		_bus_clocks[bus_index] = _frequency;
	}

	// set frequency for this instance once to the bus speed
	// the bus speed is the maximum supported by all devices on the bus,
	// as we have to prioritize performance over compatibility.
	// If a new device requires a lower clock speed, this has to be
	// manually set via "fmu i2c <bus> <clock>" before starting any
	// drivers.
	// This is necessary as automatically lowering the bus speed
	// for maximum compatibility could induce timing issues on
	// critical sensors the adopter might be unaware of.
//	I2C_SETFREQUENCY(_dev, _bus_clocks[bus_index]);
//	set_frequency(bus_clocks[bus_index]);


	// call the probe function to check whether the device is present
	ret = probe(); //

	if (ret != OK) {
		DEVICE_DEBUG("probe failed");
		goto out;
	}

	// do base class init, which will create device node, etc
	ret = CDev::init();  //字符设备初始化，创建设备节点

	if (ret != OK) {
		DEVICE_DEBUG("cdev init failed");
		goto out;
	}

	// tell the world where we are
//	DEVICE_LOG("on I2C bus %d at 0x%02x (bus: %u KHz, max: %u KHz)",
//		   _bus, _address, _bus_clocks[bus_index] / 1000, _frequency / 1000);

out:

	if ((ret != OK) && (_dev != nullptr)) {
		stm32_i2cbus_uninitialize(_dev);  //如果失败，注销i2c初始化
		_dev = nullptr;
	}

	return ret;
}

int
I2C::probe()
{
	// Assume the device is too stupid to be discoverable.
	return OK;
}


int
I2C::transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
	struct i2c_msg_s msgv[2];
	unsigned msgs;
	int ret;
	unsigned retry_count = 0;

	do {
		//	DEVICE_DEBUG("transfer out %p/%u  in %p/%u", send, send_len, recv, recv_len);

		msgs = 0;

		if (send_len > 0) {  //发送长度大于0
			msgv[msgs].frequency=_frequency;
			msgv[msgs].addr = _address;//地址
			msgv[msgs].flags = 0;//发送
			msgv[msgs].buffer = const_cast<uint8_t *>(send);//缓冲区
			msgv[msgs].length = send_len;//长度
			msgs++;
		}

		if (recv_len > 0) {//接收长度
			msgv[msgs].frequency=_frequency;
			msgv[msgs].addr = _address; //地址
			msgv[msgs].flags = I2C_M_READ;//读取
			msgv[msgs].buffer = recv;//缓冲区
			msgv[msgs].length = recv_len;//长度
			msgs++;
		}

		if (msgs == 0) {//没有消息
			return -EINVAL;
		}

		ret = I2C_TRANSFER(_dev, &msgv[0], msgs);//发送

		/* success */
		if (ret == OK) {
			break;
		}

		/* if we have already retried once, or we are going to give up, then reset the bus */
		if ((retry_count >= 1) || (retry_count >= _retries)) {
//			up_i2creset(_dev);//错误次数达到，重启设备
			I2C_RESET(_dev);
		}

	} while (retry_count++ < _retries);

	return ret;

}

int
I2C::transfer(i2c_msg_s *msgv, unsigned msgs)//多态性
{
	int ret;
	unsigned retry_count = 0;

	/* force the device address into the message vector */
	for (unsigned i = 0; i < msgs; i++) {//强行写入设备地址
		msgv[i].addr = _address;
	}


	do {
		ret = I2C_TRANSFER(_dev, msgv, msgs);

		/* success */
		if (ret == OK) {
			break;
		}

		/* if we have already retried once, or we are going to give up, then reset the bus */
		if ((retry_count >= 1) || (retry_count >= _retries)) {
//			up_i2creset(_dev);
			I2C_RESET(_dev);
		}

	} while (retry_count++ < _retries);//尝试次数

	return ret;
}

} // namespace device


