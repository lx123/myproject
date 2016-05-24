
/**
 * @file px4io.h
 *
 * General defines and structures for the PX4IO module firmware.
 */

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <board_config.h>

#include "protocol.h"

#include <systemlib/pwm_limit/pwm_limit.h>

/*
 * Constants and limits.  常量和限制
 */
#define PX4IO_SERVO_COUNT		8    //伺服输出数量
#define PX4IO_CONTROL_CHANNELS		8  //控制通道
#define PX4IO_CONTROL_GROUPS		4  //控制组
#define PX4IO_RC_INPUT_CHANNELS		18   //rc输入通道
#define PX4IO_RC_MAPPED_CONTROL_CHANNELS		8 /**< This is the maximum number of channels mapped/used */ //映射控制通道数

/*
 * Debug logging
 */

#ifdef DEBUG
# include <debug.h>
# define debug(fmt, args...)	lowsyslog(fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

/*
 * Registers.  寄存器
 */
extern uint16_t			r_page_status[];	/* PX4IO_PAGE_STATUS */    //页状态
extern uint16_t			r_page_actuators[];	/* PX4IO_PAGE_ACTUATORS */  //页执行
extern uint16_t			r_page_servos[];	/* PX4IO_PAGE_SERVOS */   //页伺服
extern uint16_t			r_page_raw_rc_input[];	/* PX4IO_PAGE_RAW_RC_INPUT */  //原始rc输入页
extern uint16_t			r_page_rc_input[];	/* PX4IO_PAGE_RC_INPUT */  //rc输入页
extern uint16_t			r_page_adc[];		/* PX4IO_PAGE_RAW_ADC_INPUT */  //原始adc输入页

extern volatile uint16_t	r_page_setup[];		/* PX4IO_PAGE_SETUP */   //设置页
extern volatile uint16_t	r_page_controls[];	/* PX4IO_PAGE_CONTROLS */  //控制页
extern uint16_t			r_page_rc_input_config[]; /* PX4IO_PAGE_RC_INPUT_CONFIG */   //rc输入控制页
extern uint16_t			r_page_servo_failsafe[]; /* PX4IO_PAGE_FAILSAFE_PWM */    //失效保护
extern uint16_t			r_page_servo_control_min[]; /* PX4IO_PAGE_CONTROL_MIN_PWM */  //控制最小pwm
extern uint16_t			r_page_servo_control_max[]; /* PX4IO_PAGE_CONTROL_MAX_PWM */  //最大pwm控制
extern uint16_t			r_page_servo_disarmed[];	/* PX4IO_PAGE_DISARMED_PWM */ //加锁

/*
 * Register aliases.寄存器别名
 *
 * Handy aliases for registers that are widely used.
 */
#define r_status_flags		r_page_status[PX4IO_P_STATUS_FLAGS]
#define r_status_alarms		r_page_status[PX4IO_P_STATUS_ALARMS]   //寄存器解锁状态

#define r_raw_rc_count		r_page_raw_rc_input[PX4IO_P_RAW_RC_COUNT]
#define r_raw_rc_values		(&r_page_raw_rc_input[PX4IO_P_RAW_RC_BASE])
#define r_raw_rc_flags		r_page_raw_rc_input[PX4IO_P_RAW_RC_FLAGS]
#define r_rc_valid			r_page_rc_input[PX4IO_P_RC_VALID]
#define r_rc_values			(&r_page_rc_input[PX4IO_P_RC_BASE])
#define r_mixer_limits 		r_page_status[PX4IO_P_STATUS_MIXER]

#define r_setup_features	r_page_setup[PX4IO_P_SETUP_FEATURES]
#define r_setup_arming		r_page_setup[PX4IO_P_SETUP_ARMING]
#define r_setup_pwm_rates	r_page_setup[PX4IO_P_SETUP_PWM_RATES]
#define r_setup_pwm_defaultrate	r_page_setup[PX4IO_P_SETUP_PWM_DEFAULTRATE]
#define r_setup_pwm_altrate	r_page_setup[PX4IO_P_SETUP_PWM_ALTRATE]

#define r_setup_rc_thr_failsafe	r_page_setup[PX4IO_P_SETUP_RC_THR_FAILSAFE_US]

#define r_setup_pwm_reverse	r_page_setup[PX4IO_P_SETUP_PWM_REVERSE]

#define r_setup_trim_roll	r_page_setup[PX4IO_P_SETUP_TRIM_ROLL]
#define r_setup_trim_pitch	r_page_setup[PX4IO_P_SETUP_TRIM_PITCH]
#define r_setup_trim_yaw	r_page_setup[PX4IO_P_SETUP_TRIM_YAW]

#define r_control_values	(&r_page_controls[0])

/*
 * System state structure.系统状态结构
 */
struct sys_state_s {

    volatile uint64_t	rc_channels_timestamp_received;  //rc通道接收时间戳
    volatile uint64_t	rc_channels_timestamp_valid;     //rc通道有效时间戳

	/**
     * Last FMU receive time, in microseconds since system boot
     * 系统启动后，最后一次fmu接收时间
	 */
	volatile uint64_t	fmu_data_received_time;

};

extern struct sys_state_s system_state;

/*
 * PWM limit structure
 */
extern pwm_limit_t pwm_limit;

/*
 * GPIO handling.
 */
#define LED_BLUE(_s)			stm32_gpiowrite(GPIO_LED1, !(_s))   //蓝色
#define LED_AMBER(_s)			stm32_gpiowrite(GPIO_LED2, !(_s))   //琥珀
#define LED_SAFETY(_s)			stm32_gpiowrite(GPIO_LED3, !(_s))   //安全开关的灯
#define LED_RING(_s)			stm32_gpiowrite(GPIO_LED4, (_s))    //未用

# define PX4IO_RELAY_CHANNELS		0
# define ENABLE_SBUS_OUT(_s)		stm32_gpiowrite(GPIO_SBUS_OENABLE, !(_s))

# define VDD_SERVO_FAULT		(!stm32_gpioread(GPIO_SERVO_FAULT_DETECT))

# define PX4IO_ADC_CHANNEL_COUNT	2
# define ADC_VSERVO			4
# define ADC_RSSI			5

#define BUTTON_SAFETY		stm32_gpioread(GPIO_BTN_SAFETY)//安全开关
//控制页面指针
#define CONTROL_PAGE_INDEX(_group, _channel) (_group * PX4IO_CONTROL_CHANNELS + _channel)

/*
 * Mixer
 */
extern void	mixer_tick(void);
extern int	mixer_handle_text(const void *buffer, size_t length);
/* Set the failsafe values of all mixed channels (based on zero throttle, controls centered) */
extern void	mixer_set_failsafe(void);   //设置失效保护所有混合通道的值

/**
 * Safety switch/LED.
 */
extern void	safety_init(void);//安全开关初始化
extern void	failsafe_led_init(void);//失效保护灯初始化

/**
 * FMU communications
 * fmu通信接口
 */
extern void	interface_init(void);//接口初始化
extern void	interface_tick(void);//接口心跳

/**
 * Register space   寄存器空间
 */
extern int	registers_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);  //设置寄存器
extern int	registers_get(uint8_t page, uint8_t offset, uint16_t **values, unsigned *num_values);      //获取寄存器

/**
 * Sensors/misc inputs传感器或者混控输入
 */
extern int	adc_init(void);    //adc初始化
extern uint16_t	adc_measure(unsigned channel);  //adc测量

/**
 * R/C receiver handling.接收机接收处理
 *
 * Input functions return true when they receive an update from the RC controller.
 */
extern void	controls_init(void);  //rc控制初始化
extern void	controls_tick(void);   //rc控制时钟

/** global debug level for isr_debug() */
extern volatile uint8_t debug_level;  //内部中断debug

/** send a debug message to the console *///发送一个debug信息给控制
extern void	isr_debug(uint8_t level, const char *fmt, ...);

/** schedule a reboot */  //安排一个重启
extern void schedule_reboot(uint32_t time_delta_usec);



