/**
 * @file gpio_led.c
 *
 * Status LED via GPIO driver.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
//#include <systemlib/systemlib.h>
#include <systemlib/err.h>
//#include <uORB/uORB.h>
//#include <uORB/topics/vehicle_status.h>
#include <poll.h>
#include <drivers/drv_gpio.h>
//#include <modules/px4iofirmware/protocol.h>
#include <sys/ioctl.h>

struct gpio_led_s {
	struct work_s work;
	int gpio_fd;
	bool use_io;
	int pin;
//	struct vehicle_status_s status;
//	int vehicle_status_sub;
	bool led_state;
	int counter;
};

static struct gpio_led_s *gpio_led_data;
static bool gpio_led_started = false;

__EXPORT int gpio_led_main(int argc, char *argv[]); //函数声明

void gpio_led_start(FAR void *arg);

void gpio_led_cycle(FAR void *arg);

int gpio_led_main(int argc, char *argv[])  //入口
{
	if (argc < 2) {
		errx(1, "usage: gpio_led {start|stop} [-p <n>]\n"
		     "\t-p <n>\tUse specified AUX OUT pin number (default: 1)"
		    );
	} else {
		if (!strcmp(argv[1], "start")) {
			if (gpio_led_started) {
				errx(1, "already running");
			}

			bool use_io = false;  //使用io模块

			/* by default use GPIO_EXT_1 on FMUv1 and GPIO_SERVO_1 on FMUv2 */
			int pin = 1;

			char pin_name[] = "AUX OUT 1";

			if (argc > 2) {//如果有参数
				if (!strcmp(argv[2], "-p")) {//如果相等
					unsigned int n = strtoul(argv[3], NULL, 10);

					if (n >= 1 && n <= 6) {
						use_io = false;  //不适用io模块
						pin = 1 << (n - 1);
						snprintf(pin_name, sizeof(pin_name), "AUX OUT %d", n);

					} else {
						errx(1, "unsupported pin: %s", argv[3]);
					}


				}
			}
			gpio_led_data = malloc(sizeof(struct gpio_led_s)); //申请内存
			memset(gpio_led_data, 0, sizeof(struct gpio_led_s));
			gpio_led_data->use_io = use_io;//使用io模块
			gpio_led_data->pin = pin;//使用的引脚
            int ret = work_queue(LPWORK, &(gpio_led_data->work), gpio_led_start, gpio_led_data, 0); //工作队列，低优先级

            if (ret != 0) {
                 errx(1, "failed to queue work: %d", ret);//工作队列失败

 			} else {
                 gpio_led_started = true; //led启动标志
                 warnx("start, using pin: %s", pin_name);//打印使用的pin名称
 				exit(0);//退出
 			}

	       } else if (!strcmp(argv[1], "stop")) {
				if (gpio_led_started) {
					gpio_led_started = false;
					warnx("stop");
					exit(0);

				} else {
					errx(1, "not running");//本来就没有运行
				}

			} else {
				errx(1, "unrecognized command '%s', only supporting 'start' or 'stop'", argv[1]); //不能识别的命令
			}
	}

}

void gpio_led_start(FAR void *arg)
{
	FAR struct gpio_led_s *priv = (FAR struct gpio_led_s *)arg;//定义一个指针，并强制转换arg

	char *gpio_dev;//设备

#if defined(PX4IO_DEVICE_PATH) //如果定义了io设备

	if (priv->use_io) {
		gpio_dev = PX4IO_DEVICE_PATH;

	} else {
		gpio_dev = PX4FMU_DEVICE_PATH;
	}

#else
	gpio_dev = PX4FMU_DEVICE_PATH;
#endif

	/* open GPIO device */
    priv->gpio_fd = open(gpio_dev, 0);//直接使用，在c语言中，设备路径号，设备已经注册

    if (priv->gpio_fd < 0) {//设备打开失败
		// TODO find way to print errors
		printf("gpio_led: GPIO device \"%s\" open fail\n", gpio_dev);
		gpio_led_started = false;
		return;
	}

	/* configure GPIO pin */
	/* px4fmu only, px4io doesn't support GPIO_SET_OUTPUT and will ignore */
	ioctl(priv->gpio_fd, GPIO_SET_OUTPUT, priv->pin);

	/* initialize vehicle status structure */
//	memset(&priv->status, 0, sizeof(priv->status));

    /* subscribe to vehicle status topic *///订阅状态
//	priv->vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* add worker to queue */
    int ret = work_queue(LPWORK, &priv->work, gpio_led_cycle, priv, 0); //工作周期纳入队列

	if (ret != 0) {
		// TODO find way to print errors
		//printf("gpio_led: failed to queue work: %d\n", ret);
		gpio_led_started = false;
		return;
	}
}

void gpio_led_cycle(FAR void *arg)
{
	FAR struct gpio_led_s *priv = (FAR struct gpio_led_s *)arg;

	/* check for status updates*/
//	bool updated;
//    orb_check(priv->vehicle_status_sub, &updated);//检查状态
//
//    if (updated) {//状态更新
//		orb_copy(ORB_ID(vehicle_status), priv->vehicle_status_sub, &priv->status);
//	}

//	/* select pattern for current status */
//	int pattern = 0;
//
//	if (priv->status.arming_state == ARMING_STATE_ARMED_ERROR) {
//		pattern = 0x2A;	// *_*_*_ fast blink (armed, error)
//
//	} else if (priv->status.arming_state == ARMING_STATE_ARMED) {
//		if (priv->status.battery_warning == VEHICLE_BATTERY_WARNING_NONE && !priv->status.failsafe) {
//			pattern = 0x3f;	// ****** solid (armed)
//
//		} else {
//			pattern = 0x3e;	// *****_ slow blink (armed, battery low or failsafe)
//		}
//
//	} else if (priv->status.arming_state == ARMING_STATE_STANDBY) {
//		pattern = 0x38;	// ***___ slow blink (disarmed, ready)
//
//	} else if (priv->status.arming_state == ARMING_STATE_STANDBY_ERROR) {
//		pattern = 0x28;	// *_*___ slow double blink (disarmed, error)
//
//	}

	/* blink pattern */
//	bool led_state_new = (pattern & (1 << priv->counter)) != 0;
//
//	if (led_state_new != priv->led_state) {
//		priv->led_state = led_state_new;
//
//		if (led_state_new) {
//			ioctl(priv->gpio_fd, GPIO_SET, priv->pin);
//
//		} else {
//			ioctl(priv->gpio_fd, GPIO_CLEAR, priv->pin);
//		}
//	}

//	priv->counter++;

//	if (priv->counter > 5) {
//		priv->counter = 0;
//	}

	/* repeat cycle at 5 Hz */
	if (gpio_led_started) {
        work_queue(LPWORK, &priv->work, gpio_led_cycle, priv, USEC2TICK(200000));//以5hz的运行速度调用自己

	} else {
		/* switch off LED on stop */
		ioctl(priv->gpio_fd, GPIO_CLEAR, priv->pin);
	}
}









