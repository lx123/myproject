/**
 * @file rgbled.cpp
 *
 * Driver for the onboard RGB LED controller (TCA62724FMG) connected via I2C.
 *挂在I2C2上
 */
#include <nuttx/config.h>
#include <px4_getopt.h>

#include <drivers/device/i2c_nuttx.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>

//#include <px4_workqueue.h>

//#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
//#include <systemlib/systemlib.h>

#include <board_config.h>

#include <drivers/drv_rgbled.h>

#define RGBLED_ONTIME 120  //灯亮时间
#define RGBLED_OFFTIME 120  //灯灭时间

#define ADDR			PX4_I2C_OBDEV_LED	/**< I2C adress of TCA62724FMG */  //设备地址
#define SUB_ADDR_START		0x01	/**< write everything (with auto-increment) */
#define SUB_ADDR_PWM0		0x81	/**< blue     (without auto-increment) */
#define SUB_ADDR_PWM1		0x82	/**< green    (without auto-increment) */
#define SUB_ADDR_PWM2		0x83	/**< red      (without auto-increment) */
#define SUB_ADDR_SETTINGS	0x84	/**< settings (without auto-increment)*/

#define SETTING_NOT_POWERSAVE	0x01	/**< power-save mode not off */
#define SETTING_ENABLE   	0x02	/**< on */  //使能


class RGBLED : public device::I2C
{
public:
	RGBLED(int bus, int rgbled);
	virtual ~RGBLED();


	virtual int		init();
	virtual int		probe();
	virtual int		info();
	virtual int		ioctl(device::file_t *filp, int cmd, unsigned long arg);

private:
	work_s			_work;  //工作队列

	rgbled_mode_t		_mode;  //枚举模式
	rgbled_pattern_t	_pattern;  //模式

	uint8_t			_r;
	uint8_t			_g;
	uint8_t			_b;
	float			_brightness;  //亮度
	float			_max_brightness;  //最大亮度

	bool			_running;  //运行
	int			_led_interval;    //间隔
	bool			_should_run;
	int			_counter;  //计数
	int			_param_sub;  //参数预定

	void 			set_color(rgbled_color_t ledcolor);  //设置颜色
	void			set_mode(rgbled_mode_t mode);  //设置模式
	void			set_pattern(rgbled_pattern_t *pattern); //亮灯方式

	static void		led_trampoline(void *arg);
	void			led();

	int			send_led_enable(bool enable); //使能
	int			send_led_rgb();
	int			get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b);
	void		update_params();  //更新参数
};

/* for now, we only support one RGBLED */
namespace  //全局名字空间
{
RGBLED *g_rgbled = nullptr; //实例
}


void rgbled_usage(); //使用方法

extern "C" __EXPORT int rgbled_main(int argc, char *argv[]);  //主函数，系统调用


RGBLED::RGBLED(int bus, int rgbled) :   //构造函数，设备号，地址
	I2C("rgbled", RGBLED0_DEVICE_PATH, bus, rgbled
	    , 100000 /* maximum speed supported */  //最大i2c速度为100k
	   ),
	_mode(RGBLED_MODE_OFF),//模式，灯灭
	_r(0),
	_g(0),
	_b(0),
	_brightness(1.0f),
	_max_brightness(1.0f),
	_running(false),
	_led_interval(0),
	_should_run(false),
	_counter(0),
	_param_sub(-1)
{
	memset(&_work, 0, sizeof(_work));  //队列初始化
	memset(&_pattern, 0, sizeof(_pattern)); //模式初始化
}

RGBLED::~RGBLED()  //析构
{
}


int
RGBLED::init()
{
	int ret;
	ret = I2C::init();  //初始化总线

	if (ret != OK) {//总线初始化失败
		return ret;
	}

	/* switch off LED on start */
	send_led_enable(false);//关灯
	send_led_rgb();//发送给芯片

	return OK;
}


int
RGBLED::probe()
{
	int ret;
	bool on, powersave;
	uint8_t r, g, b;

	/**
	   this may look strange, but is needed. There is a serial
	   EEPROM (Microchip-24aa01) on the PX4FMU-v1 that responds to
	   a bunch of I2C addresses, including the 0x55 used by this
	   LED device. So we need to do enough operations to be sure
	   we are talking to the right device. These 3 operations seem
	   to be enough, as the 3rd one consistently fails if no
	   RGBLED is on the bus.
	 */

	unsigned prevretries = _retries;
	_retries = 4;

	if ((ret = get(on, powersave, r, g, b)) != OK ||
	    (ret = send_led_enable(false) != OK) ||
	    (ret = send_led_enable(false) != OK)) {
		return ret;
	}

	_retries = prevretries;

	return ret;
}

int
RGBLED::info()
{
	int ret;
	bool on, powersave;
	uint8_t r, g, b;

	ret = get(on, powersave, r, g, b);

	if (ret == OK) {
		/* we don't care about power-save mode */
//		DEVICE_LOG("state: %s", on ? "ON" : "OFF");
//		DEVICE_LOG("red: %u, green: %u, blue: %u", (unsigned)r, (unsigned)g, (unsigned)b);

	} else {
		warnx("failed to read led");
	}

	return ret;
}


int
RGBLED::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = ENOTTY;

	switch (cmd) {
	case RGBLED_SET_RGB:
		/* set the specified color */
		_r = ((rgbled_rgbset_t *) arg)->red;
		_g = ((rgbled_rgbset_t *) arg)->green;
		_b = ((rgbled_rgbset_t *) arg)->blue;
		send_led_rgb();
		return OK;

	case RGBLED_SET_COLOR:
		/* set the specified color name */
		set_color((rgbled_color_t)arg);
		send_led_rgb();
		return OK;

	case RGBLED_SET_MODE:
		/* set the specified mode */
		set_mode((rgbled_mode_t)arg);
		return OK;

	case RGBLED_SET_PATTERN:
		/* set a special pattern */
		set_pattern((rgbled_pattern_t *)arg);
		return OK;

	default:
		/* see if the parent class can make any use of it */

		ret = CDev::ioctl(filp, cmd, arg);
		break;
	}

	return ret;
}


void
RGBLED::led_trampoline(void *arg)
{
	RGBLED *rgbl = reinterpret_cast<RGBLED *>(arg);

	rgbl->led();
}


/**
 * Main loop function
 */
void
RGBLED::led()
{
	if (!_should_run) {
		_running = false;
		return;
	}

	if (_param_sub < 0) {
//		_param_sub = orb_subscribe(ORB_ID(parameter_update));
	}

//	if (_param_sub >= 0) {
//		bool updated = false;
////		orb_check(_param_sub, &updated);
//
//		if (updated) {
//			parameter_update_s pupdate;
//			orb_copy(ORB_ID(parameter_update), _param_sub, &pupdate);
//			update_params();
//			// Immediately update to change brightness
//			send_led_rgb();
//		}
//	}

	switch (_mode) {
	case RGBLED_MODE_BLINK_SLOW:
	case RGBLED_MODE_BLINK_NORMAL:
	case RGBLED_MODE_BLINK_FAST:
		if (_counter >= 2) {
			_counter = 0;
		}

		send_led_enable(_counter == 0);

		break;

	case RGBLED_MODE_BREATHE:

		if (_counter >= 62) {
			_counter = 0;
		}

		int n;

		if (_counter < 32) {
			n = _counter;

		} else {
			n = 62 - _counter;
		}

		_brightness = n * n / (31.0f * 31.0f);
		send_led_rgb();
		break;

	case RGBLED_MODE_PATTERN:

		/* don't run out of the pattern array and stop if the next frame is 0 */
		if (_counter >= RGBLED_PATTERN_LENGTH || _pattern.duration[_counter] <= 0) {
			_counter = 0;
		}

		set_color(_pattern.color[_counter]);
		send_led_rgb();
		_led_interval = _pattern.duration[_counter];
		break;

	default:
		break;
	}

	_counter++;

	/* re-queue ourselves to run again later */
	work_queue(LPWORK, &_work, (worker_t)&RGBLED::led_trampoline, this, _led_interval);  //工作队列
}

/**
 * Parse color constant and set _r _g _b values
 */
void
RGBLED::set_color(rgbled_color_t color)
{
	switch (color) {
	case RGBLED_COLOR_OFF:
		_r = 0;
		_g = 0;
		_b = 0;
		break;

	case RGBLED_COLOR_RED:
		_r = 255;
		_g = 0;
		_b = 0;
		break;

	case RGBLED_COLOR_YELLOW:
		_r = 255;
		_g = 200;
		_b = 0;
		break;

	case RGBLED_COLOR_PURPLE:
		_r = 255;
		_g = 0;
		_b = 255;
		break;

	case RGBLED_COLOR_GREEN:
		_r = 0;
		_g = 255;
		_b = 0;
		break;

	case RGBLED_COLOR_BLUE:
		_r = 0;
		_g = 0;
		_b = 255;
		break;

	case RGBLED_COLOR_WHITE:
		_r = 255;
		_g = 255;
		_b = 255;
		break;

	case RGBLED_COLOR_AMBER:
		_r = 255;
		_g = 80;
		_b = 0;
		break;

	case RGBLED_COLOR_DIM_RED:
		_r = 90;
		_g = 0;
		_b = 0;
		break;

	case RGBLED_COLOR_DIM_YELLOW:
		_r = 80;
		_g = 30;
		_b = 0;
		break;

	case RGBLED_COLOR_DIM_PURPLE:
		_r = 45;
		_g = 0;
		_b = 45;
		break;

	case RGBLED_COLOR_DIM_GREEN:
		_r = 0;
		_g = 90;
		_b = 0;
		break;

	case RGBLED_COLOR_DIM_BLUE:
		_r = 0;
		_g = 0;
		_b = 90;
		break;

	case RGBLED_COLOR_DIM_WHITE:
		_r = 30;
		_g = 30;
		_b = 30;
		break;

	case RGBLED_COLOR_DIM_AMBER:
		_r = 80;
		_g = 20;
		_b = 0;
		break;

	default:
		warnx("color unknown");
		break;
	}
}

/**
 * Set mode, if mode not changed has no any effect (doesn't reset blinks phase)
 */
void
RGBLED::set_mode(rgbled_mode_t mode)
{
	if (mode != _mode) {
		_mode = mode;

		switch (mode) {
		case RGBLED_MODE_OFF:
			_should_run = false;
			send_led_enable(false);
			break;

		case RGBLED_MODE_ON:
			_brightness = 1.0f;
			send_led_rgb();
			send_led_enable(true);
			break;

		case RGBLED_MODE_BLINK_SLOW:
			_should_run = true;
			_counter = 0;
			_led_interval = 2000;
			_brightness = 1.0f;
			send_led_rgb();
			break;

		case RGBLED_MODE_BLINK_NORMAL:
			_should_run = true;
			_counter = 0;
			_led_interval = 500;
			_brightness = 1.0f;
			send_led_rgb();
			break;

		case RGBLED_MODE_BLINK_FAST:
			_should_run = true;
			_counter = 0;
			_led_interval = 100;
			_brightness = 1.0f;
			send_led_rgb();
			break;

		case RGBLED_MODE_BREATHE:
			_should_run = true;
			_counter = 0;
			_led_interval = 25;
			send_led_enable(true);
			break;

		case RGBLED_MODE_PATTERN:
			_should_run = true;
			_counter = 0;
			_brightness = 1.0f;
			send_led_enable(true);
			break;

		default:
			warnx("mode unknown");
			break;
		}

		/* if it should run now, start the workq */
		if (_should_run && !_running) { //运行
			_running = true;
			work_queue(LPWORK, &_work, (worker_t)&RGBLED::led_trampoline, this, 1);//低优先级
		}

	}
}

/**
 * Set pattern for PATTERN mode, but don't change current mode
 */
void
RGBLED::set_pattern(rgbled_pattern_t *pattern)
{
	memcpy(&_pattern, pattern, sizeof(rgbled_pattern_t));
}

/**
 * Sent ENABLE flag to LED driver
 */
int
RGBLED::send_led_enable(bool enable)
{
	uint8_t settings_byte = 0;

	if (enable) {
		settings_byte |= SETTING_ENABLE; //使能
	}

	settings_byte |= SETTING_NOT_POWERSAVE; //不保存

	const uint8_t msg[2] = { SUB_ADDR_SETTINGS, settings_byte};

	return transfer(msg, sizeof(msg), nullptr, 0);  //失能芯片
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED::send_led_rgb()
{
	/* To scale from 0..255 -> 0..15 shift right by 4 bits */
	const uint8_t msg[6] = {
		SUB_ADDR_PWM0, static_cast<uint8_t>((_b >> 4) * _brightness * _max_brightness + 0.5f),
		SUB_ADDR_PWM1, static_cast<uint8_t>((_g >> 4) * _brightness * _max_brightness + 0.5f),
		SUB_ADDR_PWM2, static_cast<uint8_t>((_r >> 4) * _brightness * _max_brightness + 0.5f)
	};
	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
RGBLED::get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b)
{
	uint8_t result[2];
	int ret;

	ret = transfer(nullptr, 0, &result[0], 2);

	if (ret == OK) {
		on = result[0] & SETTING_ENABLE;
		powersave = !(result[0] & SETTING_NOT_POWERSAVE);
		/* XXX check, looks wrong */
		r = (result[0] & 0x0f) << 4;
		g = (result[1] & 0xf0);
		b = (result[1] & 0x0f) << 4;
	}

	return ret;
}

void
RGBLED::update_params()
{
	int32_t maxbrt = 15;
//	param_get(param_find("LED_RGB_MAXBRT"), &maxbrt);
	maxbrt = maxbrt > 15 ? 15 : maxbrt;
	maxbrt = maxbrt <  0 ?  0 : maxbrt;

	// A minimum of 2 "on" steps is required for breathe effect
	if (maxbrt == 1) {
		maxbrt = 2;
	}

	_max_brightness = maxbrt / 15.0f;
}

void
rgbled_usage()
{
	warnx("missing command: try 'start', 'test', 'info', 'off', 'stop', 'rgb 30 40 50'");
	warnx("options:");
	warnx("    -b i2cbus (%d)", PX4_I2C_BUS_LED);
	warnx("    -a addr (0x%x)", ADDR);
}

int
rgbled_main(int argc, char *argv[])  //入口函数
{
	int i2cdevice = -1; //设备号
	int rgbledadr = ADDR; /* 7bit */  //地址

	int ch;

	/* jump over start/off/etc and look at options first */  //跳过start/off/etc，先搞定参数
	int myoptind = 1; //自定义选项
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "a:b:", &myoptind, &myoptarg)) != EOF) {  //寻找其中参数
		switch (ch) {
		case 'a':
			rgbledadr = strtol(myoptarg, NULL, 0);  //设备地址
			break;

		case 'b':
			i2cdevice = strtol(myoptarg, NULL, 0); //设备总线号
			break;

		default:
			rgbled_usage();
			return 1;
		}
	}//参数寻找完毕

	if (myoptind >= argc) { //参数完毕，说明使用错误
		rgbled_usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	int fd;
	int ret;

	if (!strcmp(verb, "start")) { //选项
		if (g_rgbled != nullptr) {
			warnx("already started");
			return 1;
		}

		if (i2cdevice == -1) {
			// try the external bus first  尝试使用外部总线
			i2cdevice = PX4_I2C_BUS_EXPANSION;
			g_rgbled = new RGBLED(PX4_I2C_BUS_EXPANSION, rgbledadr); //创建对象

			if (g_rgbled != nullptr && OK != g_rgbled->init()) { //初始化
				delete g_rgbled;
				g_rgbled = nullptr;
			}

			if (g_rgbled == nullptr) {//失败
				// fall back to default bus
				if (PX4_I2C_BUS_LED == PX4_I2C_BUS_EXPANSION) {//如果led挂在的总线对应
					warnx("no RGB led on bus #%d", i2cdevice);
					return 1;
				}

				i2cdevice = PX4_I2C_BUS_LED;//设备号
			}
		}

		if (g_rgbled == nullptr) {//如果对象为空
			g_rgbled = new RGBLED(i2cdevice, rgbledadr);//设备所在的总线

			if (g_rgbled == nullptr) {//对象实例化失败
				warnx("new failed");
				return 1;
			}

			if (OK != g_rgbled->init()) {//初始化
				delete g_rgbled;
				g_rgbled = nullptr;
				warnx("no RGB led on bus #%d", i2cdevice);
				return 1;
			}
		}

		return 0;
	}

	/* need the driver past this point */
	if (g_rgbled == nullptr) {//如果设备失败
		warnx("not started");
		rgbled_usage();
		return 1;
	}

	if (!strcmp(verb, "test")) {  //测试
		fd = px4_open(RGBLED0_DEVICE_PATH, 0); //打开设备

		if (fd == -1) {
			warnx("Unable to open " RGBLED0_DEVICE_PATH);
			return 1;
		}

		rgbled_pattern_t pattern = { {RGBLED_COLOR_RED, RGBLED_COLOR_GREEN, RGBLED_COLOR_BLUE, RGBLED_COLOR_WHITE, RGBLED_COLOR_OFF, RGBLED_COLOR_OFF},
			{500, 500, 500, 500, 1000, 0 }	// "0" indicates end of pattern
		};

		ret = px4_ioctl(fd, RGBLED_SET_PATTERN, (unsigned long)&pattern);
		ret = px4_ioctl(fd, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_PATTERN);

		px4_close(fd);
		return ret;
	}

	if (!strcmp(verb, "info")) {
		g_rgbled->info();
		return 0;
	}

	if (!strcmp(verb, "off") || !strcmp(verb, "stop")) {
		fd = px4_open(RGBLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			warnx("Unable to open " RGBLED0_DEVICE_PATH);
			return 1;
		}

		ret = px4_ioctl(fd, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_OFF);
		px4_close(fd);

		/* delete the rgbled object if stop was requested, in addition to turning off the LED. */
		if (!strcmp(verb, "stop")) {  //停止
			delete g_rgbled;
			g_rgbled = nullptr;
			return 0;
		}

		return ret;
	}

	if (!strcmp(verb, "rgb")) {
		if (argc < 5) {
			warnx("Usage: rgbled rgb <red> <green> <blue>");
			return 1;
		}

		fd = px4_open(RGBLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			warnx("Unable to open " RGBLED0_DEVICE_PATH);
			return 1;
		}

		rgbled_rgbset_t v;
		v.red   = strtol(argv[2], NULL, 0);
		v.green = strtol(argv[3], NULL, 0);
		v.blue  = strtol(argv[4], NULL, 0);
		ret = px4_ioctl(fd, RGBLED_SET_RGB, (unsigned long)&v);
		ret = px4_ioctl(fd, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_ON);
		px4_close(fd);
		return ret;
	}

	rgbled_usage();
	return 1;
}




