/**
 * @file px4io.c
 * Top-level logic for the PX4IO module.
 * 1-8 pwm输出都是这个芯片
 * 整个io芯片就干两件事，一外部输入 ppm/sbus等解码，二。八个通道混控输出pwm 三/与fmu通信，类似传感器的伪寄存器访问方式
 */
#include <nuttx/config.h>  //包含系统和板子的头文件
#include <nuttx/arch.h>  //nuttx-export的include中

#include <stdio.h>	// required for task_create
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <signal.h>
#include <crc32.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include <systemlib/perf_counter.h>  //性能测量工具
#include <systemlib/pwm_limit/pwm_limit.h>  //pwm限制

#include <stm32_uart.h>

#define DEBUG
#include "io.h"

__EXPORT int user_start(int argc, char *argv[]); //主程序开始

extern void up_cxxinitialize(void);//c++初始化

struct sys_state_s 	system_state;//系统状态

static struct hrt_call serial_dma_call;//串口dma调用

pwm_limit_t pwm_limit;

/*
 * a set of debug buffers to allow us to send debug information from ISRs
 * 一系列debug缓存器允许我们发送来自中断服务debug信息
 */

static volatile uint32_t msg_counter;//消息计数
static volatile uint32_t last_msg_counter;//最后一次消息计数
static volatile uint8_t msg_next_out, msg_next_in;  //消息下一个输出，消息下一个输入

/*
 * WARNING: too large buffers here consume the memory required
 * for mixer handling. Do not allocate more than 80 bytes for
 * output.
 * 警告：在处理混空时会消耗大量的缓冲器，不要申请超过80字节用于输出
 */
#define NUM_MSG 1
static char msg[NUM_MSG][40];//消息组

static void heartbeat_blink(void);//心跳闪灯
static void ring_blink(void);//声音

/*
 * add a debug message to be printed on the console
 * 添加一个debug信息到控制台并打印出来
 */
void
isr_debug(uint8_t level, const char *fmt, ...)
{
    if (level > r_page_setup[PX4IO_P_SETUP_SET_DEBUG]) {//按等级发送
		return;
	}

	va_list ap;
	va_start(ap, fmt);
	vsnprintf(msg[msg_next_in], sizeof(msg[0]), fmt, ap);
	va_end(ap);
	msg_next_in = (msg_next_in + 1) % NUM_MSG;
	msg_counter++;
}

/*
 * show all pending debug messages
 * 显示所有挂起的debug信息
 */
static void
show_debug_messages(void)
{
	if (msg_counter != last_msg_counter) {
		uint32_t n = msg_counter - last_msg_counter;

		if (n > NUM_MSG) { n = NUM_MSG; }

		last_msg_counter = msg_counter;

		while (n--) {
			debug("%s", msg[msg_next_out]);
			msg_next_out = (msg_next_out + 1) % NUM_MSG;
		}
	}
}

static void
heartbeat_blink(void)//心跳灯
{
	static bool heartbeat = false;
	LED_BLUE(heartbeat = !heartbeat);  //蓝色为心跳，
}

static void
ring_blink(void)
{
//#ifdef GPIO_LED4
//
//	if (/* IO armed */ (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)
//			   /* and FMU is armed */ && (r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)) {
//		LED_RING(1);
//		return;
//	}
//
//	// XXX this led code does have
//	// intentionally a few magic numbers.
//	const unsigned max_brightness = 118;
//
//	static unsigned counter = 0;
//	static unsigned brightness = max_brightness;
//	static unsigned brightness_counter = 0;
//	static unsigned on_counter = 0;
//
//	if (brightness_counter < max_brightness) {
//
//		bool on = ((on_counter * 100) / brightness_counter + 1) <= ((brightness * 100) / max_brightness + 1);
//
//		// XXX once led is PWM driven,
//		// remove the ! in the line below
//		// to return to the proper breathe
//		// animation / pattern (currently inverted)
//		LED_RING(!on);
//		brightness_counter++;
//
//		if (on) {
//			on_counter++;
//		}
//
//	} else {
//
//		if (counter >= 62) {
//			counter = 0;
//		}
//
//		int n;
//
//		if (counter < 32) {
//			n = counter;
//
//		} else {
//			n = 62 - counter;
//		}
//
//		brightness = (n * n) / 8;
//		brightness_counter = 0;
//		on_counter = 0;
//		counter++;
//	}
//
//#endif
}

static uint64_t reboot_time;//重启时间

/**
   schedule a reboot in time_delta_usec microseconds  在时间内安排重启
 */
void schedule_reboot(uint32_t time_delta_usec)
{
	reboot_time = hrt_absolute_time() + time_delta_usec;  //当前时间加延迟时间
}

/**
   check for a scheduled reboot
 */
static void check_reboot(void)
{
	if (reboot_time != 0 && hrt_absolute_time() > reboot_time) {
        up_systemreset();//上位系统重置？？
	}
}
//计算固件的crc，判断是否需要升级固件
static void
calculate_fw_crc(void)
{
#define APP_SIZE_MAX 0xf000  //应用程序最大空间
#define APP_LOAD_ADDRESS 0x08001000  //应用程序装载地址
	// compute CRC of the current firmware
	uint32_t sum = 0;  //保存当前固件的计算结果

	for (unsigned p = 0; p < APP_SIZE_MAX; p += 4) {  //对整个app空间进行轮询，一行8位，一次性取4行32位
		uint32_t bytes = *(uint32_t *)(p + APP_LOAD_ADDRESS);  //读取32位
		sum = crc32part((uint8_t *)&bytes, sizeof(bytes), sum);  //crc计算，将32位转换成8位无符号，系统内部计算
	}

    r_page_setup[PX4IO_P_SETUP_CRC]   = sum & 0xFFFF;//取低16位 ，保存crc的值 设置页
    r_page_setup[PX4IO_P_SETUP_CRC + 1] = sum >> 16;//取高16位
}

int
user_start(int argc, char *argv[])  //主入口
{
	/* configure the first 8 PWM outputs (i.e. all of them) */
    up_pwm_servo_init(0xff);//设置pwm，实参为通道掩码一位代表一个通道

	/* run C++ ctors before we go any further */
	up_cxxinitialize();

    /* reset all to zero */  //系统状态清零
    memset(&system_state, 0, sizeof(system_state));//包含rc时间戳，和rc失效时间戳

	/* configure the high-resolution time/callout interface */
    //配置高分辨率的时间/调用接口。timer1设置
	hrt_init();  //在board中设置

	/* calculate our fw CRC so FMU can decide if we need to update */
    calculate_fw_crc();//计算当前固件的crc信息，这样fmu就知道我们的固件是否需要更新，保存到设置页里面

	/*
	 * Poll at 1ms intervals for received bytes that have not triggered
     * a DMA event.
     * 1ms间隔轮询接收数据，如果没有dma事件触发
	 */
#ifdef CONFIG_ARCH_DMA  //系统中定义
	hrt_call_every(&serial_dma_call, 1000, 1000, (hrt_callout)stm32_serial_dma_poll, NULL);  //轮询串口 线程2，dma轮询 1000hz
#endif

    /* print some startup info */  //打印一些启动信息，io模块启动
	lowsyslog("\nPX4IO: starting\n");

	/* default all the LEDs to off while we start */
	LED_AMBER(false); //关闭琥珀led
    LED_BLUE(false);//蓝色led
    LED_SAFETY(false);//安全开关
//#ifdef GPIO_LED4
//	LED_RING(false);
//#endif

    /* turn on servo power (if supported) *///打开伺服的电源，如果支持
#ifdef POWER_SERVO
	POWER_SERVO(true);
#endif

    /* turn off S.Bus out (if supported) */  //关闭sbus输出，如果支持
#ifdef ENABLE_SBUS_OUT
	ENABLE_SBUS_OUT(false);
#endif

    /* start the safety switch handler *///启动安全开关处理
	safety_init();  //线程1，频率10hz

	/* initialise the control inputs */
    controls_init();//外部控制初始化，sbus ppm dsm

	/* set up the ADC */
    adc_init();  //模拟量

	/* start the FMU interface */
    interface_init(); //开启与fmu的接口，串口2dma自动处理 线程2,串口dma轮询

    /* add a performance counter for mixing */  //添加一个混控执行计数，测量消耗的时间
	perf_counter_t mixer_perf = perf_alloc(PC_ELAPSED, "mix");

	/* add a performance counter for controls */  //控制性能计数
	perf_counter_t controls_perf = perf_alloc(PC_ELAPSED, "controls");

    /* and one for measuring the loop rate */  //测量两次事件的间隔
	perf_counter_t loop_perf = perf_alloc(PC_INTERVAL, "loop");

    struct mallinfo minfo = mallinfo();//内存调用信息
	lowsyslog("MEM: free %u, largest %u\n", minfo.mxordblk, minfo.fordblks);

    /* initialize PWM limit lib */  //pwm限制
	pwm_limit_init(&pwm_limit);

	/*
	 *    P O L I C E    L I G H T S
	 *
	 * Not enough memory, lock down.
	 *
	 * We might need to allocate mixers later, and this will
	 * ensure that a developer doing a change will notice
	 * that he just burned the remaining RAM with static
	 * allocations. We don't want him to be able to
	 * get past that point. This needs to be clearly
	 * documented in the dev guide.
	 *
	 */
    if (minfo.mxordblk < 600) { //内存信息，可用内存小于600

        lowsyslog("ERR: not enough MEM"); //没有足够的内存
		bool phase = false;

        while (true) { //琥珀色的灯和蓝色的交互闪烁，死循环

			if (phase) {
				LED_AMBER(true);  //琥珀和蓝色交替闪烁
				LED_BLUE(false);

			} else {
				LED_AMBER(false);
				LED_BLUE(true);
			}

			up_udelay(250000);  //250ms，500ms频率

			phase = !phase;
		}
	}

	/* Start the failsafe led init */
    failsafe_led_init(); //失效保护led  线程3，失效保护8hz

	/*
	 * Run everything in a tight loop.  在一个紧凑的循环中运行一切
	 */

	uint64_t last_debug_time = 0;  //最后一次debug时间
	uint64_t last_heartbeat_time = 0;  //最后一次心跳时间

    for (;;) {  //主循环，如果内存足够

		/* track the rate at which the loop is running */
		perf_count(loop_perf); //主循环计数

		/* kick the mixer */  //打开混控
		perf_begin(mixer_perf);  //执行一次混控消耗时间
		mixer_tick();  //混控滴答 重要之一
		perf_end(mixer_perf);

		/* kick the control inputs */  //打开控制输入
		perf_begin(controls_perf);  //计算时间花费时间
		controls_tick();  //控制输入 重要之二
		perf_end(controls_perf);

		if ((hrt_absolute_time() - last_heartbeat_time) > 250 * 1000) { //对时间进行轮询
			last_heartbeat_time = hrt_absolute_time();
            heartbeat_blink();//心跳灯闪烁，蓝色
		}

		ring_blink();  //led4，不存在

		check_reboot(); //检查是否需要复位，轮询检查复位

		/* check for debug activity (default: none) */
		show_debug_messages();   //检查debug是否活动

		/* post debug state at ~1Hz - this is via an auxiliary serial port
		 * DEFAULTS TO OFF!
		 */
		if (hrt_absolute_time() - last_debug_time > (1000 * 1000)) {

			isr_debug(1, "d:%u s=0x%x a=0x%x f=0x%x m=%u",
				  (unsigned)r_page_setup[PX4IO_P_SETUP_SET_DEBUG],
				  (unsigned)r_status_flags,
				  (unsigned)r_setup_arming,
				  (unsigned)r_setup_features,
				  (unsigned)mallinfo().mxordblk);
			last_debug_time = hrt_absolute_time();
		}
    }//结束主循环
}

