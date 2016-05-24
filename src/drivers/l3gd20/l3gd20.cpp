/**
 * @file l3gd20.cpp
 * Driver for the ST L3GD20 MEMS and L3GD20H mems gyros connected via SPI.
 *
 * Note: With the exception of the self-test feature, the ST L3G4200D is
 *       also supported by this driver.
 */

#include <nuttx/config.h>
#include <board_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/spi.h>
#include <drivers/drv_gyro.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>

#include <board_config.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>
#include<platforms/px4_defines.h>

#define L3GD20_DEVICE_PATH "/dev/l3gd20"   //设备地址，注册设备时使用

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/* Orientation on board */  //传感器在板子上的方向
#define SENSOR_BOARD_ROTATION_000_DEG	0
#define SENSOR_BOARD_ROTATION_090_DEG	1
#define SENSOR_BOARD_ROTATION_180_DEG	2
#define SENSOR_BOARD_ROTATION_270_DEG	3

/* SPI protocol address bits */
#define DIR_READ				(1<<7)   //spi读取
#define DIR_WRITE				(0<<7)    //spi写入
#define ADDR_INCREMENT				(1<<6)  //地址增加

/* register addresses */  //传感器寄存器地址
#define ADDR_WHO_AM_I			0x0F
#define WHO_I_AM_H 				0xD7
#define WHO_I_AM				0xD4
#define WHO_I_AM_L3G4200D		0xD3	/* for L3G4200D */

#define ADDR_CTRL_REG1			0x20
#define REG1_RATE_LP_MASK			0xF0 /* Mask to guard partial register update */

/* keep lowpass low to avoid noise issues */
#define RATE_95HZ_LP_25HZ		((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_190HZ_LP_25HZ		((0<<7) | (1<<6) | (0<<5) | (1<<4))
#define RATE_190HZ_LP_50HZ		((0<<7) | (1<<6) | (1<<5) | (0<<4))
#define RATE_190HZ_LP_70HZ		((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define RATE_380HZ_LP_20HZ		((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define RATE_380HZ_LP_25HZ		((1<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_380HZ_LP_50HZ		((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define RATE_380HZ_LP_100HZ		((1<<7) | (0<<6) | (1<<5) | (1<<4))
#define RATE_760HZ_LP_30HZ		((1<<7) | (1<<6) | (0<<5) | (0<<4))
#define RATE_760HZ_LP_35HZ		((1<<7) | (1<<6) | (0<<5) | (1<<4))
#define RATE_760HZ_LP_50HZ		((1<<7) | (1<<6) | (1<<5) | (0<<4))
#define RATE_760HZ_LP_100HZ		((1<<7) | (1<<6) | (1<<5) | (1<<4))

#define ADDR_CTRL_REG2			0x21
#define ADDR_CTRL_REG3			0x22
#define ADDR_CTRL_REG4			0x23
#define REG4_RANGE_MASK				0x30 /* Mask to guard partial register update */
#define RANGE_250DPS				(0<<4)
#define RANGE_500DPS				(1<<4)
#define RANGE_2000DPS				(3<<4)

#define ADDR_CTRL_REG5			0x24
#define ADDR_REFERENCE			0x25
#define ADDR_OUT_TEMP			0x26
#define ADDR_STATUS_REG			0x27
#define ADDR_OUT_X_L			0x28
#define ADDR_OUT_X_H			0x29
#define ADDR_OUT_Y_L			0x2A
#define ADDR_OUT_Y_H			0x2B
#define ADDR_OUT_Z_L			0x2C
#define ADDR_OUT_Z_H			0x2D
#define ADDR_FIFO_CTRL_REG		0x2E
#define ADDR_FIFO_SRC_REG		0x2F
#define ADDR_INT1_CFG			0x30
#define ADDR_INT1_SRC			0x31
#define ADDR_INT1_TSH_XH		0x32
#define ADDR_INT1_TSH_XL		0x33
#define ADDR_INT1_TSH_YH		0x34
#define ADDR_INT1_TSH_YL		0x35
#define ADDR_INT1_TSH_ZH		0x36
#define ADDR_INT1_TSH_ZL		0x37
#define ADDR_INT1_DURATION		0x38
#define ADDR_LOW_ODR			0x39


/* Internal configuration values */  //内部配置值
#define REG1_POWER_NORMAL			(1<<3)
#define REG1_Z_ENABLE				(1<<2)
#define REG1_Y_ENABLE				(1<<1)
#define REG1_X_ENABLE				(1<<0)

#define REG4_BDU				(1<<7)
#define REG4_BLE				(1<<6)
//#define REG4_SPI_3WIRE			(1<<0)

#define REG5_FIFO_ENABLE			(1<<6)
#define REG5_REBOOT_MEMORY			(1<<7)

#define STATUS_ZYXOR				(1<<7)
#define STATUS_ZOR				(1<<6)
#define STATUS_YOR				(1<<5)
#define STATUS_XOR				(1<<4)
#define STATUS_ZYXDA				(1<<3)
#define STATUS_ZDA				(1<<2)
#define STATUS_YDA				(1<<1)
#define STATUS_XDA				(1<<0)

#define FIFO_CTRL_BYPASS_MODE			(0<<5)
#define FIFO_CTRL_FIFO_MODE			(1<<5)
#define FIFO_CTRL_STREAM_MODE			(1<<6)
#define FIFO_CTRL_STREAM_TO_FIFO_MODE		(3<<5)
#define FIFO_CTRL_BYPASS_TO_STREAM_MODE		(1<<7)

#define L3GD20_DEFAULT_RATE			760   //默认速率
#define L3G4200D_DEFAULT_RATE			800
#define L3GD20_MAX_OUTPUT_RATE			280  //最大输出速率
#define L3GD20_DEFAULT_RANGE_DPS		2000
#define L3GD20_DEFAULT_FILTER_FREQ		30   //默认滤波器频率
#define L3GD20_TEMP_OFFSET_CELSIUS		40   //温度偏移，单位摄氏度

#define L3GD20_MAX_OFFSET			0.45f /**< max offset: 25 degrees/s */

#ifdef PX4_SPI_BUS_EXT   //如果外部spi总线存在
#define EXTERNAL_BUS PX4_SPI_BUS_EXT  //挂在外部spi4总线上
#else
#define EXTERNAL_BUS 0
#endif

#ifndef SENSOR_BOARD_ROTATION_DEFAULT  //默认芯片在板上的角度
#define SENSOR_BOARD_ROTATION_DEFAULT		SENSOR_BOARD_ROTATION_270_DEG
#endif

/*
  we set the timer interrupt to run a bit faster than the desired  设置时间中断比期望的采样频率快，然后复制使用的数据
  sample rate and then throw away duplicates using the data ready bit.
  This time reduction is enough to cope with worst case timing jitter  这个时间消耗
  due to other timers
 */
#define L3GD20_TIMER_REDUCTION				600

extern "C" { __EXPORT int l3gd20_main(int argc, char *argv[]); }  //主函数入口，申明能让c调用

//传感器类，继承spi类，，类声明
class L3GD20 : public device::SPI  //继承父类为SPI
{
public:
	L3GD20(int bus, const char *path, spi_dev_e device, enum Rotation rotation); //构造函数
	virtual ~L3GD20();

	virtual int		init();  //初始化

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg); //io控制

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

	// print register dump 打印释放的寄存器
	void			print_registers();

	// trigger an error
	void			test_error();

protected:
	virtual int		probe();

private:

	struct hrt_call		_call;
	unsigned		_call_interval;

	ringbuffer::RingBuffer	*_reports;  //环形缓冲区

	struct gyro_scale	_gyro_scale;   //陀螺仪缩放
	float			_gyro_range_scale;  //陀螺仪量程缩放
	float			_gyro_range_rad_s; //堕落已量程 弧度/秒
	orb_advert_t		_gyro_topic;   //陀螺仪会话
	int			_orb_class_instance;  //会话标号
	int			_class_instance;      //类标号

	unsigned		_current_rate;    //当前速率
	unsigned		_orientation;     //方向

	unsigned		_read;            //读取

	perf_counter_t		_sample_perf;
	perf_counter_t		_errors;
	perf_counter_t		_bad_registers;
	perf_counter_t		_duplicates;

	uint8_t			_register_wait;

	math::LowPassFilter2p	_gyro_filter_x;  //2阶低通滤波器
	math::LowPassFilter2p	_gyro_filter_y;
	math::LowPassFilter2p	_gyro_filter_z;

	Integrator		_gyro_int;     //陀螺仪积分，声明一个实例

	/* true if an L3G4200D is detected */
	bool	_is_l3g4200d;  //检测到l3gd200d标志位

	enum Rotation		_rotation;  //旋转

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define L3GD20_NUM_CHECKED_REGISTERS 8  //检查寄存器数量
	static const uint8_t	_checked_registers[L3GD20_NUM_CHECKED_REGISTERS];  //检查寄存器
	uint8_t			_checked_values[L3GD20_NUM_CHECKED_REGISTERS];    //检查数值
	uint8_t			_checked_next;  //下一个

	/**
	 * Start automatic measurement.
	 */
	void			start();  //启动自动测量

	/**
	 * Stop automatic measurement.
	 */
	void			stop(); //停止自动测量

	/**
	 * Reset the driver
	 */
	void			reset();  //重启驱动

	/**
	 * disable I2C on the chip
	 */
	void			disable_i2c();  //禁止i2c连接芯片

	/**
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool			is_external() { return (_bus == EXTERNAL_BUS); }  //判断是挂载内部总线spi1上还是挂在外部spi4上

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(void *arg);  //测量跳床

	/**
	 * check key registers for correct values
	 */
	void			check_registers(void);  //检查关键寄存器是否是准确的值

	/**
	 * Fetch measurements from the sensor and update the report ring.
	 */
	void			measure();  //从传感器获取数值，并存入环形缓冲器

	/**
	 * Read a register from the L3GD20
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);  //读取寄存器

	/**
	 * Write a register in the L3GD20
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);  //写入寄存器

	/**
	 * Modify a register in the L3GD20
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);  //修改一个寄存器

	/**
	 * Write a register in the L3GD20, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);  //写入一个寄存器，并检查值

	/**
	 * Set the L3GD20 measurement range.
	 *
	 * @param max_dps	The measurement range is set to permit reading at least
	 *			this rate in degrees per second.
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_range(unsigned max_dps);  //设置量程

	/**
	 * Set the L3GD20 internal sampling frequency.
	 *
	 * @param frequency	The internal sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			set_samplerate(unsigned frequency);  //设置内部采样频率

	/**
	 * Set the lowpass filter of the driver
	 *
	 * @param samplerate	The current samplerate
	 * @param frequency	The cutoff frequency for the lowpass filter
	 */
	void			set_driver_lowpass_filter(float samplerate, float bandwidth);  //设置低通滤波器

	/**
	 * Self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			self_test();  //自检

	/* this class does not allow copying */  //这个类不允许复制
	L3GD20(const L3GD20 &);
	L3GD20 operator=(const L3GD20 &);
};


/*
  list of registers that will be checked in check_registers(). Note  需要检查的寄存器列表
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t L3GD20::_checked_registers[L3GD20_NUM_CHECKED_REGISTERS] = { ADDR_WHO_AM_I,    //数组初始化
									   ADDR_CTRL_REG1,
									   ADDR_CTRL_REG2,
									   ADDR_CTRL_REG3,
									   ADDR_CTRL_REG4,
									   ADDR_CTRL_REG5,
									   ADDR_FIFO_CTRL_REG,
									   ADDR_LOW_ODR
									 };


//构造函数
L3GD20::L3GD20(int bus, const char *path, spi_dev_e device, enum Rotation rotation) :
	SPI("L3GD20", path, bus, device, SPIDEV_MODE3, //初始化spi设备
	    11 * 1000 * 1000 /* will be rounded to 10.4 MHz, within margins for L3GD20 */),
	_call{},
	_call_interval(0),
	_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_gyro_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1),
	_current_rate(0),
	_orientation(SENSOR_BOARD_ROTATION_DEFAULT),
	_read(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "l3gd20_read")),
	_errors(perf_alloc(PC_COUNT, "l3gd20_errors")),  //事件计数器，错误计数
	_bad_registers(perf_alloc(PC_COUNT, "l3gd20_bad_registers")), //事件计数器，错误寄存器计数
	_duplicates(perf_alloc(PC_COUNT, "l3gd20_duplicates")),   //事件计数器，数据复制计数
	_register_wait(0),
	_gyro_filter_x(L3GD20_DEFAULT_RATE, L3GD20_DEFAULT_FILTER_FREQ),  //陀螺仪滤波默认，数据速率760，滤波频率30
	_gyro_filter_y(L3GD20_DEFAULT_RATE, L3GD20_DEFAULT_FILTER_FREQ),
	_gyro_filter_z(L3GD20_DEFAULT_RATE, L3GD20_DEFAULT_FILTER_FREQ),
	_gyro_int(1000000 / L3GD20_MAX_OUTPUT_RATE, true),  //陀螺仪初始化积分器类，第一个为复位时间间隔280hz，第二个为锥形补偿使能
	_is_l3g4200d(false),  //没有用到l3gd200d传感器
	_rotation(rotation),
	_checked_next(0)
{
	// enable debug() calls
	_debug_enabled = true;

	_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_L3GD20;  //设备类型

	// default scale factors  默认缩放因子，默认无偏移，无缩放
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;
}

	//析构函数
L3GD20::~L3GD20()
{
	/* make sure we are truly inactive */
	stop();   //确保真正停止活动，取消hrt的轮询

	/* free any existing reports */
	if (_reports != nullptr) {  //如果环形缓冲区不为空
		delete _reports;
	}

	if (_class_instance != -1) {  //类编号不为空
		unregister_class_devname(GYRO_BASE_DEVICE_PATH, _class_instance);   //注销类设备
	}

	/* delete the perf counter */  //删除所有的性能计数器
	perf_free(_sample_perf);    //采样计数
	perf_free(_errors);         //错误计数
	perf_free(_bad_registers);  //错误寄存器计数
	perf_free(_duplicates);     //复制计数
}

int
L3GD20::init()   //初始化
{
	int ret = ERROR;

	/* do SPI init (and probe) first */
	if (SPI::init() != OK) { //spi初始化
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(gyro_report));  //创建环形缓冲器2个gyro_report，在uorb中

	if (_reports == nullptr) {
		goto out;
	}

	_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);  //注册类设备，并返回编号

	reset();  //重启芯片

	measure();  //测量

	/* advertise sensor topic, measure manually to initialize valid report */
	struct gyro_report grp;  //发布传感器会话，手动发布初始化有效值
	_reports->get(&grp); //获取缓冲区中的数值

	_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
					  &_orb_class_instance, (is_external()) ? ORB_PRIO_VERY_HIGH : ORB_PRIO_DEFAULT);  //r如果是外部总线，优先级非常高

	if (_gyro_topic == nullptr) {  //会话为空
		DEVICE_DEBUG("failed to create sensor_gyro publication");
	}

	ret = OK;
out:
	return ret;
}

int
L3GD20::probe()
{
	/* read dummy value to void to clear SPI statemachine on sensor */
	(void)read_reg(ADDR_WHO_AM_I);

	bool success = false;
	uint8_t v = 0;

	/* verify that the device is attached and functioning, accept
	 * L3GD20, L3GD20H and L3G4200D */
	if ((v = read_reg(ADDR_WHO_AM_I)) == WHO_I_AM) {  //读取芯片编码，判断使用的是哪个芯片
		_orientation = SENSOR_BOARD_ROTATION_DEFAULT;
		success = true;

	} else if ((v = read_reg(ADDR_WHO_AM_I)) == WHO_I_AM_H) {  //使用的是l3gd20h
		_orientation = SENSOR_BOARD_ROTATION_180_DEG;
		success = true;  //检测成功标志

	} else if ((v = read_reg(ADDR_WHO_AM_I)) == WHO_I_AM_L3G4200D) {
		/* Detect the L3G4200D used on AeroCore */
		_is_l3g4200d = true;
		_orientation = SENSOR_BOARD_ROTATION_DEFAULT;
		success = true;
	}

	if (success) {
		_checked_values[0] = v;
		return OK;
	}

	return -EIO;
}

ssize_t
L3GD20::read(struct file *filp, char *buffer, size_t buflen)  //读取数据，文件句柄，环形缓冲区，和缓冲区大小
{
	unsigned count = buflen / sizeof(struct gyro_report); //读取数量，缓冲区大小应该是数据结构的整数倍
	struct gyro_report *gbuf = reinterpret_cast<struct gyro_report *>(buffer);  //发布结构体指针
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {  //如果缓冲区太小
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_call_interval > 0) {  //如果自动测量使能

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the measurement code while we are doing this;
		 * we are careful to avoid racing with it.
		 */
		while (count--) {  //按数量获取数据
			if (_reports->get(gbuf)) {
				ret += sizeof(*gbuf);
				gbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN; //如果没有数据出来，警告调用者
	}

	/* manual measurement */
	_reports->flush();  //手动测量
	measure();

	/* measurement will have generated a report, copy it out */
	if (_reports->get(gbuf)) {
		ret = sizeof(*gbuf);
	}

	return ret;
}

int
L3GD20::ioctl(struct file *filp, int cmd, unsigned long arg)  //io控制器
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: { //传感器ioc轮询速率
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT:  //默认的传感器轮询速率
				if (_is_l3g4200d) {  //检测到l3gd20
					return ioctl(filp, SENSORIOCSPOLLRATE, L3G4200D_DEFAULT_RATE);
				}

				return ioctl(filp, SENSORIOCSPOLLRATE, L3GD20_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */  //轮询速率，单位Hz
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);  //准备启动

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;  //频率转换成时间，ms

					/* check against maximum sane rate */
					if (ticks < 1000) {  //检查是否超过最大速率
						return -EINVAL;
					}

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call_interval = ticks;  //每次测量间隔

					_call.period = _call_interval - L3GD20_TIMER_REDUCTION; //减去传感器时间消耗

					/* adjust filters */  //自适应滤波
					float cutoff_freq_hz = _gyro_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f / ticks;  //采样速率
					set_driver_lowpass_filter(sample_rate, cutoff_freq_hz); //设置低通滤波器

					/* if we need to start the poll state machine, do it */  //启动轮询状态机
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:   //传感器io控制器G轮询速率
		if (_call_interval == 0) {  //如果没有设置自动测量
			return SENSOR_POLLRATE_MANUAL;  //手动轮询
		}

		return 1000000 / _call_interval;

	case SENSORIOCSQUEUEDEPTH: {  //传感器io控制器S队列深度
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = up_irq_save();

			if (!_reports->resize(arg)) {
				up_irq_restore(flags);
				return -ENOMEM;
			}

			up_irq_restore(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:   //传感器IO控制器G队列深度
		return _reports->size();

	case SENSORIOCRESET:  //传感器复位
		reset();
		return OK;

	case GYROIOCSSAMPLERATE:   //设置采样速率
		return set_samplerate(arg);

	case GYROIOCGSAMPLERATE:   //获得当前采样速率
		return _current_rate;

	case GYROIOCSLOWPASS: {  //低通滤波器设置
			float cutoff_freq_hz = arg;
			float sample_rate = 1.0e6f / _call_interval;
			set_driver_lowpass_filter(sample_rate, cutoff_freq_hz);

			return OK;
		}

	case GYROIOCGLOWPASS:  //返回当前低通滤波器状态
		return static_cast<int>(_gyro_filter_x.get_cutoff_freq());

	case GYROIOCSSCALE:  //设置S缩放
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_scale *) arg, sizeof(_gyro_scale));
		return OK;

	case GYROIOCGSCALE:  //设置G缩放
		/* copy scale out */
		memcpy((struct gyro_scale *) arg, &_gyro_scale, sizeof(_gyro_scale));
		return OK;

	case GYROIOCSRANGE:    //设置S量程，单位 °/s
		/* arg should be in dps */
		return set_range(arg);

	case GYROIOCGRANGE:     //设置G量程，转换成°/s，和圈数
		/* convert to dps and round */
		return (unsigned long)(_gyro_range_rad_s * 180.0f / M_PI_F + 0.5f);

	case GYROIOCSELFTEST:   //陀螺仪自己测试
		return self_test();

	default:
		/* give it to the superclass */  //否则给到父类
		return SPI::ioctl(filp, cmd, arg);
	}
}

uint8_t
L3GD20::read_reg(unsigned reg)  //读取寄存器
{
	uint8_t cmd[2];

	cmd[0] = reg | DIR_READ;  //读
	cmd[1] = 0; //获得的数据

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];  //返回获得的数据
}

void
L3GD20::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;  //写
	cmd[1] = value; //写入的数据

	transfer(cmd, nullptr, sizeof(cmd));
}

void
L3GD20::write_checked_reg(unsigned reg, uint8_t value)  //写入并检查寄存器
{
	write_reg(reg, value);  //写入寄存器

	for (uint8_t i = 0; i < L3GD20_NUM_CHECKED_REGISTERS; i++) {  //遍历所有寄存器
		if (reg == _checked_registers[i]) {  //如果寄存器找到
			_checked_values[i] = value;  //设置检查的值
		}
	}
}


void
L3GD20::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)  //修改寄存器
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

int
L3GD20::set_range(unsigned max_dps)  //设置量程
{
	uint8_t bits = REG4_BDU;
	float new_range_scale_dps_digit;
	float new_range;

	if (max_dps == 0) {
		max_dps = 2000;  //最大速度，2000°/s
	}

	if (max_dps <= 250) {
		new_range = 250;
		bits |= RANGE_250DPS;
		new_range_scale_dps_digit = 8.75e-3f;

	} else if (max_dps <= 500) {
		new_range = 500;
		bits |= RANGE_500DPS;
		new_range_scale_dps_digit = 17.5e-3f;

	} else if (max_dps <= 2000) {
		new_range = 2000;
		bits |= RANGE_2000DPS;
		new_range_scale_dps_digit = 70e-3f;

	} else {
		return -EINVAL;
	}

	_gyro_range_rad_s = new_range / 180.0f * M_PI_F;  //量程转换为弧度/s
	_gyro_range_scale = new_range_scale_dps_digit / 180.0f * M_PI_F;  //量程缩放，转换为弧度/s
	write_checked_reg(ADDR_CTRL_REG4, bits);   //设置并检查寄存器

	return OK;
}

int
L3GD20::set_samplerate(unsigned frequency)   //设置采样率
{
	uint8_t bits = REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;

	if (frequency == 0 || frequency == GYRO_SAMPLERATE_DEFAULT) {  //如果采样率为0，或者为默认的采样率
		frequency = _is_l3g4200d ? 800 : 760;  //760hz
	}

	/*
	 * Use limits good for H or non-H models. Rates are slightly different
	 * for L3G4200D part but register settings are the same.  l3gd20和l3gd20h的采样率要低一些
	 */
	if (frequency <= 100) {
		_current_rate = _is_l3g4200d ? 100 : 95;
		bits |= RATE_95HZ_LP_25HZ;

	} else if (frequency <= 200) {
		_current_rate = _is_l3g4200d ? 200 : 190;
		bits |= RATE_190HZ_LP_50HZ;

	} else if (frequency <= 400) {
		_current_rate = _is_l3g4200d ? 400 : 380;
		bits |= RATE_380HZ_LP_50HZ;

	} else if (frequency <= 800) {
		_current_rate = _is_l3g4200d ? 800 : 760;
		bits |= RATE_760HZ_LP_50HZ;

	} else {
		return -EINVAL;
	}

	write_checked_reg(ADDR_CTRL_REG1, bits);  //写入并检查寄存器

	return OK;
}

void
L3GD20::set_driver_lowpass_filter(float samplerate, float bandwidth)  //设置驱动器低通滤波器
{
	_gyro_filter_x.set_cutoff_frequency(samplerate, bandwidth);  //设置采样频率和带宽
	_gyro_filter_y.set_cutoff_frequency(samplerate, bandwidth);
	_gyro_filter_z.set_cutoff_frequency(samplerate, bandwidth);
}

void
L3GD20::start()
{
	/* make sure we are stopped first */
	stop(); //先确定是停止的，取消hrt队列中的数据

	/* reset the report ring */
	_reports->flush(); //重置发布环形缓冲器

	/* start polling at the specified rate */  //指定速率轮询
	hrt_call_every(&_call,  //回调入口
		       1000,  //延时时间
		       _call_interval - L3GD20_TIMER_REDUCTION,  //轮询间隔
		       (hrt_callout)&L3GD20::measure_trampoline, this);  //回调函数
}

void
L3GD20::stop()
{
	hrt_cancel(&_call);   //取消hrt回调
}

void
L3GD20::disable_i2c(void)  //禁止I2C
{
	uint8_t retries = 10;  //尝试次数

	while (retries--) {
		// add retries
		uint8_t a = read_reg(0x05);
		write_reg(0x05, (0x20 | a));

		if (read_reg(0x05) == (a | 0x20)) {
			// this sets the I2C_DIS bit on the
			// L3GD20H. The l3gd20 datasheet doesn't
			// mention this register, but it does seem to
			// accept it.
			write_checked_reg(ADDR_LOW_ODR, 0x08);
			return;
		}
	}

	DEVICE_DEBUG("FAILED TO DISABLE I2C");
}

void
L3GD20::reset()  //复位
{
	// ensure the chip doesn't interpret any other bus traffic as I2C
	disable_i2c(); //禁用芯片的i2c端口

	/* set default configuration */
	write_checked_reg(ADDR_CTRL_REG1,
			  REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE); //写入并检查寄存器
	write_checked_reg(ADDR_CTRL_REG2, 0);		/* disable high-pass filters */
	write_checked_reg(ADDR_CTRL_REG3, 0x08);        /* DRDY enable */
	write_checked_reg(ADDR_CTRL_REG4, REG4_BDU);
	write_checked_reg(ADDR_CTRL_REG5, 0);
	write_checked_reg(ADDR_CTRL_REG5, REG5_FIFO_ENABLE);		/* disable wake-on-interrupt */

	/* disable FIFO. This makes things simpler and ensures we
	 * aren't getting stale data. It means we must run the hrt
	 * callback fast enough to not miss data. */
	write_checked_reg(ADDR_FIFO_CTRL_REG, FIFO_CTRL_BYPASS_MODE);

	set_samplerate(0); // 760Hz or 800Hz
	set_range(L3GD20_DEFAULT_RANGE_DPS); //设置量程2000°/s
	set_driver_lowpass_filter(L3GD20_DEFAULT_RATE, L3GD20_DEFAULT_FILTER_FREQ);  //设置低通滤波器，采样速率760hz，滤波频率30

	_read = 0;
}

void
L3GD20::measure_trampoline(void *arg)  //测量跳板
{
	L3GD20 *dev = (L3GD20 *)arg;  //实例

	/* make another measurement */
	dev->measure();  //启动另一个测量
}

void
L3GD20::check_registers(void)   //检查寄存器
{
	uint8_t v;

	if ((v = read_reg(_checked_registers[_checked_next])) != _checked_values[_checked_next]) {
		/*
		  if we get the wrong value then we know the SPI bus
		  or sensor is very sick. We set _register_wait to 20
		  and wait until we have seen 20 good values in a row
		  before we consider the sensor to be OK again.
		 */
		perf_count(_bad_registers);

		/*
		  try to fix the bad register value. We only try to
		  fix one per loop to prevent a bad sensor hogging the
		  bus. We skip zero as that is the WHO_AM_I, which
		  is not writeable
		 */
		if (_checked_next != 0) {
			write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
		}

		_register_wait = 20;
	}

	_checked_next = (_checked_next + 1) % L3GD20_NUM_CHECKED_REGISTERS;
}

void
L3GD20::measure()  //测量
{
	/* status register and data as read back from the device */
#pragma pack(push, 1)  //状态寄存器和数据
	struct {
		uint8_t		cmd;  //命令号，后面跟着数据
		int8_t		temp;  //温度
		uint8_t		status; //状态
		int16_t		x;      //三个轴
		int16_t		y;
		int16_t		z;
	} raw_report;  //原始数据，直接定义数据结构
#pragma pack(pop)

	gyro_report report;    //发布数据结构体

	/* start the performance counter */
	perf_begin(_sample_perf);  //开始性能测试计数器

	check_registers();  //检查寄存器

	/* fetch data from the sensor */
	memset(&raw_report, 0, sizeof(raw_report));  //初始化
	raw_report.cmd = ADDR_OUT_TEMP | DIR_READ | ADDR_INCREMENT;  //发布地址
	transfer((uint8_t *)&raw_report, (uint8_t *)&raw_report, sizeof(raw_report)); //获取原始数据，通过spi

	if (!(raw_report.status & STATUS_ZYXDA)) {  //比较状态，如果读取错误
		perf_end(_sample_perf);  //停止采样性能计数器
		perf_count(_duplicates);
		return;
	}

	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.  缩放原始数据到国际标准单位
	 * 2) Subtract static offset (in SI units)  //在国际标准单位下减去静态偏移
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor  缩放静态校准值，线性动态获得的因数
	 *
	 * Note: the static sensor offset is the number the sensor outputs  静态传感器偏移数值是正常情况下0输入
	 * 	 at a nominally 'zero' input. Therefore the offset has to  因此这个偏移不得不被减去
	 * 	 be subtracted.
	 *
	 *	 Example: A gyro outputs a value of 74 at zero angular rate
	 *	 	  the offset is 74 from the origin and subtracting
	 *		  74 from all measurements centers them around zero.
	 */
	report.timestamp = hrt_absolute_time();  //当前绝对时间，时间戳
	report.error_count = perf_event_count(_bad_registers); //错误性能计数器

	switch (_orientation) { //依据芯片的方向改变，只有xy方向会改变

	case SENSOR_BOARD_ROTATION_000_DEG:
		/* keep axes in place */
		report.x_raw = raw_report.x;
		report.y_raw = raw_report.y;
		break;

	case SENSOR_BOARD_ROTATION_090_DEG:
		/* swap x and y */
		report.x_raw = raw_report.y;
		report.y_raw = raw_report.x;
		break;

	case SENSOR_BOARD_ROTATION_180_DEG:
		/* swap x and y and negate both */
		report.x_raw = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
		report.y_raw = ((raw_report.y == -32768) ? 32767 : -raw_report.y);
		break;

	case SENSOR_BOARD_ROTATION_270_DEG:
		/* swap x and y and negate y */
		report.x_raw = raw_report.y;
		report.y_raw = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
		break;
	}

	report.z_raw = raw_report.z;  //z方向不变

	report.temperature_raw = raw_report.temp; //温度转存

	float xraw_f = report.x_raw;
	float yraw_f = report.y_raw;
	float zraw_f = report.z_raw;

	// apply user specified rotation  应用用户制定的旋转
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);  //应用用户指定的旋转，在lib/conversion中，，，在命令行中指定，默认不旋转

	float xin = ((xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;  //前面是量程缩放，后面为国际单位化
	float yin = ((yraw_f * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	float zin = ((zraw_f * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	report.x = _gyro_filter_x.apply(xin);   //低通 滤波  滤波后的数据没了
	report.y = _gyro_filter_y.apply(yin);
	report.z = _gyro_filter_z.apply(zin);

	math::Vector<3> gval(xin, yin, zin); //矢量
	math::Vector<3> gval_integrated;  //向量积分寄存器

	bool gyro_notify = _gyro_int.put(report.timestamp, gval, gval_integrated, report.integral_dt); //陀螺仪积分，返回积分状态
	report.x_integral = gval_integrated(0); //存入积分数据
	report.y_integral = gval_integrated(1);
	report.z_integral = gval_integrated(2);

	report.temperature = L3GD20_TEMP_OFFSET_CELSIUS - raw_report.temp; //温度，计算实际温度

	report.scaling = _gyro_range_scale; //量程缩放
	report.range_rad_s = _gyro_range_rad_s; //量程 弧度/s

	_reports->force(&report); //强制放入环形缓冲器

	if (gyro_notify) { //积分通过
		/* notify anyone waiting for data */  //通知等待数据
		poll_notify(POLLIN);  //轮询通知，发布新的轮询时间

		/* publish for subscribers */  //发布时间
		if (!(_pub_blocked)) {
			/* publish it */
			orb_publish(ORB_ID(sensor_gyro), _gyro_topic, &report);  //发布数据
		}
	}

	_read++;  //读取计数

	/* stop the perf counter */
	perf_end(_sample_perf); //停止性能计数器
}

void
L3GD20::print_info()   //打印相关信息
{
	printf("gyro reads:          %u\n", _read);  //读取数量
	perf_print_counter(_sample_perf);   //采样性能
	perf_print_counter(_errors);    //错误计数
	perf_print_counter(_bad_registers);  //错误寄存器
	perf_print_counter(_duplicates);      //复制计数
	_reports->print_info("report queue");    //发布队列
	::printf("checked_next: %u\n", _checked_next);  //检查下一个

	for (uint8_t i = 0; i < L3GD20_NUM_CHECKED_REGISTERS; i++) {  //打印检查的寄存器值
		uint8_t v = read_reg(_checked_registers[i]);

		if (v != _checked_values[i]) {
			::printf("reg %02x:%02x should be %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_values[i]);
		}
	}
}

void
L3GD20::print_registers()  //打印寄存器
{
	printf("L3GD20 registers\n");

	for (uint8_t reg = 0; reg <= 0x40; reg++) {  //打印所有的寄存器
		uint8_t v = read_reg(reg);
		printf("%02x:%02x ", (unsigned)reg, (unsigned)v);

		if ((reg + 1) % 16 == 0) {
			printf("\n");
		}
	}

	printf("\n");
}

void
L3GD20::test_error()  //测试错误
{
	// trigger a deliberate error  触发一个故意的错误
	write_reg(ADDR_CTRL_REG3, 0);
}

int
L3GD20::self_test()  //自检
{
	/* evaluate gyro offsets, complain if offset -> zero or larger than 25 dps */  //估计陀螺仪的偏移，如果偏移为0或者大于25°/s
	if (fabsf(_gyro_scale.x_offset) > L3GD20_MAX_OFFSET || fabsf(_gyro_scale.x_offset) < 0.000001f) {
		return 1;
	}

	if (fabsf(_gyro_scale.x_scale - 1.0f) > 0.3f) {
		return 1;
	}

	if (fabsf(_gyro_scale.y_offset) > L3GD20_MAX_OFFSET || fabsf(_gyro_scale.y_offset) < 0.000001f) {
		return 1;
	}

	if (fabsf(_gyro_scale.y_scale - 1.0f) > 0.3f) {
		return 1;
	}

	if (fabsf(_gyro_scale.z_offset) > L3GD20_MAX_OFFSET || fabsf(_gyro_scale.z_offset) < 0.000001f) {
		return 1;
	}

	if (fabsf(_gyro_scale.z_scale - 1.0f) > 0.3f) {
		return 1;
	}

	return 0;
}

/**
 * Local functions in support of the shell command.
 * 名词空间中的项目
 */
namespace l3gd20
{

L3GD20	*g_dev;

void	usage();
void	start(bool external_bus, enum Rotation rotation);
void	test();
void	reset();
void	info();
void	regdump();
void	test_error();

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * started or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation)  //是否为外部中线spi4，传感器旋转多少度
{
	int fd;

	if (g_dev != nullptr) {  //如果实例不为空
		errx(0, "already started");
	}

	/* create the driver */
	if (external_bus) {
#ifdef PX4_SPI_BUS_EXT
		g_dev = new L3GD20(PX4_SPI_BUS_EXT, L3GD20_DEVICE_PATH, (spi_dev_e)PX4_SPIDEV_EXT_GYRO, rotation);
#else
		errx(0, "External SPI not available");
#endif

	} else {
		g_dev = new L3GD20(PX4_SPI_BUS_SENSORS, L3GD20_DEVICE_PATH, (spi_dev_e)PX4_SPIDEV_GYRO, rotation);  //创建一个对象，spi1，设备地址，spi1总线上的芯片标号，旋转度数
	}

	if (g_dev == nullptr) {  //对象创建失败
		goto fail;
	}

	if (OK != g_dev->init()) {   //对象初始化
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(L3GD20_DEVICE_PATH, O_RDONLY);  //只读方式打开设备，设置轮询默认速率，开始自动数据收集,在创建类中注册的spi设备

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {  //设置轮询速率
		goto fail;
	}

	close(fd);//关闭设备

	exit(0);
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	int fd_gyro = -1;
	struct gyro_report g_report;
	ssize_t sz;

	/* get the driver */
	fd_gyro = open(L3GD20_DEVICE_PATH, O_RDONLY);  //打开设备，读取

	if (fd_gyro < 0) {  //读取失败
		err(1, "%s open failed", L3GD20_DEVICE_PATH);
	}

	/* reset to manual polling */  //复位到手动轮询
	if (ioctl(fd_gyro, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0) {
		err(1, "reset to manual polling");
	}

	/* do a simple demand read */
	sz = read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {  //读取文件
		err(1, "immediate gyro read failed");
	}

	warnx("gyro x: \t% 9.5f\trad/s", (double)g_report.x); //弧度/s
	warnx("gyro y: \t% 9.5f\trad/s", (double)g_report.y);
	warnx("gyro z: \t% 9.5f\trad/s", (double)g_report.z);
	warnx("temp: \t%d\tC", (int)g_report.temperature);
	warnx("gyro x: \t%d\traw", (int)g_report.x_raw);
	warnx("gyro y: \t%d\traw", (int)g_report.y_raw);
	warnx("gyro z: \t%d\traw", (int)g_report.z_raw);
	warnx("temp: \t%d\traw", (int)g_report.temperature_raw);
	warnx("gyro range: %8.4f rad/s (%d deg/s)", (double)g_report.range_rad_s,
	      (int)((g_report.range_rad_s / M_PI_F) * 180.0f + 0.5f));

	if (ioctl(fd_gyro, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "reset to default polling");
	}

	close(fd_gyro);

	/* XXX add poll-rate tests here too */
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()  //复位驱动
{
	int fd = open(L3GD20_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "accel pollrate reset failed");
	}

	close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running\n");
	}

	printf("state @ %p\n", g_dev);  //打印当前信息
	g_dev->print_info();

	exit(0);
}

/**
 * Dump the register information  打印寄存器信息
 */
void
regdump(void)
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("regdump @ %p\n", g_dev);
	g_dev->print_registers();

	exit(0);
}

/**
 * trigger an error
 */
void
test_error(void)
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("regdump @ %p\n", g_dev);
	g_dev->test_error();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset', 'testerror' or 'regdump'");
	warnx("options:");
	warnx("    -X    (external bus)");
	warnx("    -R rotation");
}

} // namespace
//c语言，  外部
int
l3gd20_main(int argc, char *argv[])
{
	bool external_bus = false;
	int ch;
	enum Rotation rotation = ROTATION_NONE;  //默认不旋转

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "XR:")) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true; //外部总线spi4
			break;

		case 'R':
			rotation = (enum Rotation)atoi(optarg);  //旋转
			break;

		default:
			l3gd20::usage();  //否则打印使用信息
			exit(0);
		}
	}

	const char *verb = argv[optind];  //除了上面的变量

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		l3gd20::start(external_bus, rotation); //外部输入的旋转
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		l3gd20::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		l3gd20::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		l3gd20::info();
	}

	/*
	 * Print register information.
	 */
	if (!strcmp(verb, "regdump")) {
		l3gd20::regdump();
	}

	/*
	 * trigger an error
	 */
	if (!strcmp(verb, "testerror")) {
		l3gd20::test_error();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset', 'info', 'testerror' or 'regdump'");
}




