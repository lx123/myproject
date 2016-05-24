
/**
 * @file registers.c
 *
 * Implementation of the PX4IO register space.
 */

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <systemlib/systemlib.h>
#include <stm32_pwr.h>
#include <rc/dsm.h>

#include "io.h"
#include "protocol.h"

static int	registers_set_one(uint8_t page, uint8_t offset, uint16_t value);   //设置一个寄存器
static void	pwm_configure_rates(uint16_t map, uint16_t defaultrate, uint16_t altrate);  //pwm配置速率

/**
 * PAGE 0
 *
 * Static configuration parameters.  静态配置参数
 */
static const uint16_t	r_page_config[] = {
	[PX4IO_P_CONFIG_PROTOCOL_VERSION]	= PX4IO_PROTOCOL_VERSION,
	[PX4IO_P_CONFIG_HARDWARE_VERSION]	= 2,
	[PX4IO_P_CONFIG_BOOTLOADER_VERSION]	= 3,	/* XXX hardcoded magic number */
	[PX4IO_P_CONFIG_MAX_TRANSFER]		= 64,	/* XXX hardcoded magic number */
	[PX4IO_P_CONFIG_CONTROL_COUNT]		= PX4IO_CONTROL_CHANNELS,  //控制通道数
	[PX4IO_P_CONFIG_ACTUATOR_COUNT]		= PX4IO_SERVO_COUNT,  //伺服数量
	[PX4IO_P_CONFIG_RC_INPUT_COUNT]		= PX4IO_RC_INPUT_CHANNELS,   //输入通道
	[PX4IO_P_CONFIG_ADC_INPUT_COUNT]	= PX4IO_ADC_CHANNEL_COUNT,   //adc通道数量
	[PX4IO_P_CONFIG_RELAY_COUNT]		= PX4IO_RELAY_CHANNELS,
};

/**
 * PAGE 1
 *
 * Status values.  状态值
 */
uint16_t		r_page_status[] = {
	[PX4IO_P_STATUS_FREEMEM]		= 0,  //空余ram
	[PX4IO_P_STATUS_CPULOAD]		= 0,
	[PX4IO_P_STATUS_FLAGS]			= 0,
	[PX4IO_P_STATUS_ALARMS]			= 0,   //解锁
	[PX4IO_P_STATUS_VBATT]			= 0,   //电池电压
	[PX4IO_P_STATUS_IBATT]			= 0,   //电池电流
	[PX4IO_P_STATUS_VSERVO]			= 0,
	[PX4IO_P_STATUS_VRSSI]			= 0,
	[PX4IO_P_STATUS_PRSSI]			= 0,
	[PX4IO_P_STATUS_MIXER]			= 0,
};

/**
 * PAGE 2
 *
 * Post-mixed actuator values.
 */
uint16_t 		r_page_actuators[PX4IO_SERVO_COUNT];  //投递的混合执行机构的值

/**
 * PAGE 3
 *
 * Servo PWM values
 */
uint16_t		r_page_servos[PX4IO_SERVO_COUNT];  //伺服pwm的值

/**
 * PAGE 4
 *
 * Raw RC input
 */
uint16_t		r_page_raw_rc_input[] = {  //原始rc输入
	[PX4IO_P_RAW_RC_COUNT]			= 0,
	[PX4IO_P_RAW_RC_FLAGS]			= 0,
	[PX4IO_P_RAW_RC_NRSSI]			= 0,
	[PX4IO_P_RAW_RC_DATA]			= 0,
	[PX4IO_P_RAW_FRAME_COUNT]		= 0,
	[PX4IO_P_RAW_LOST_FRAME_COUNT]		= 0,
	[PX4IO_P_RAW_RC_BASE ...(PX4IO_P_RAW_RC_BASE + PX4IO_RC_INPUT_CHANNELS)] = 0
};

/**
 * PAGE 5
 *
 * Scaled/routed RC input
 */
uint16_t		r_page_rc_input[] = {  //缩放/路径rc输入
	[PX4IO_P_RC_VALID]			= 0,
	[PX4IO_P_RC_BASE ...(PX4IO_P_RC_BASE + PX4IO_RC_MAPPED_CONTROL_CHANNELS)] = 0
};

/**
 * Scratch page; used for registers that are constructed as-read.
 *
 * PAGE 6 Raw ADC input.  原始adc输入
 * PAGE 7 PWM rate maps.  pwm速率映射
 */
uint16_t		r_page_scratch[32];

/**
 * PAGE 100
 *
 * Setup registers
 */
volatile uint16_t	r_page_setup[] = {  //设置寄存器
	/* default to RSSI ADC functionality */  //默认为adc功能获取接收信号指示器
	[PX4IO_P_SETUP_FEATURES]		= PX4IO_P_SETUP_FEATURES_ADC_RSSI,

	[PX4IO_P_SETUP_ARMING]			= (PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE),
	[PX4IO_P_SETUP_PWM_RATES]		= 0,
	[PX4IO_P_SETUP_PWM_DEFAULTRATE]		= 50,
	[PX4IO_P_SETUP_PWM_ALTRATE]		= 200,
	/* this is unused, but we will pad it for readability (the compiler pads it automatically) */
	[PX4IO_P_SETUP_RELAYS_PAD]		= 0,
#ifdef ADC_VSERVO
	[PX4IO_P_SETUP_VSERVO_SCALE]		= 10000,
#else
	[PX4IO_P_SETUP_VBATT_SCALE]		= 10000,
#endif
	[PX4IO_P_SETUP_SET_DEBUG]		= 0,
	[PX4IO_P_SETUP_REBOOT_BL]		= 0,
	[PX4IO_P_SETUP_CRC ...(PX4IO_P_SETUP_CRC + 1)] = 0,
	[PX4IO_P_SETUP_RC_THR_FAILSAFE_US] = 0,
	[PX4IO_P_SETUP_PWM_REVERSE] = 0,
	[PX4IO_P_SETUP_TRIM_ROLL] = 0,
	[PX4IO_P_SETUP_TRIM_PITCH] = 0,
	[PX4IO_P_SETUP_TRIM_YAW] = 0
};


#define PX4IO_P_SETUP_FEATURES_VALID	(PX4IO_P_SETUP_FEATURES_SBUS1_OUT | \
		PX4IO_P_SETUP_FEATURES_SBUS2_OUT | \
		PX4IO_P_SETUP_FEATURES_ADC_RSSI | \
		PX4IO_P_SETUP_FEATURES_PWM_RSSI)

#define PX4IO_P_SETUP_ARMING_VALID	(PX4IO_P_SETUP_ARMING_FMU_ARMED | \
		PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK | \
		PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK | \
		PX4IO_P_SETUP_ARMING_IO_ARM_OK | \
		PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM | \
		PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE | \
		PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED | \
		PX4IO_P_SETUP_ARMING_LOCKDOWN | \
		PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE | \
		PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE | \
		PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE)
#define PX4IO_P_SETUP_RATES_VALID	((1 << PX4IO_SERVO_COUNT) - 1)
#define PX4IO_P_SETUP_RELAYS_VALID	((1 << PX4IO_RELAY_CHANNELS) - 1)

/**
 * PAGE 101
 *
 * Control values from the FMU.
 */
volatile uint16_t	r_page_controls[PX4IO_CONTROL_GROUPS * PX4IO_CONTROL_CHANNELS];  //来自fmu的控制值

/*
 * PAGE 102 does not have a buffer.
 */

/**
 * PAGE 103
 *
 * R/C channel input configuration.
 */
uint16_t		r_page_rc_input_config[PX4IO_RC_INPUT_CHANNELS * PX4IO_P_RC_CONFIG_STRIDE];  //rc通道输入配置

/* valid options */
#define PX4IO_P_RC_CONFIG_OPTIONS_VALID	(PX4IO_P_RC_CONFIG_OPTIONS_REVERSE | PX4IO_P_RC_CONFIG_OPTIONS_ENABLED)   //有效选项

/*
 * PAGE 104 uses r_page_servos.
 */

/**
 * PAGE 105
 *
 * Failsafe servo PWM values  失效保护pwm值
 *
 * Disable pulses as default.  停止脉冲
 */
uint16_t		r_page_servo_failsafe[PX4IO_SERVO_COUNT] = { 0, 0, 0, 0, 0, 0, 0, 0 };

/**
 * PAGE 106
 *
 * minimum PWM values when armed  解锁后最小pwm值
 *
 */
uint16_t		r_page_servo_control_min[PX4IO_SERVO_COUNT] = { PWM_DEFAULT_MIN, PWM_DEFAULT_MIN, PWM_DEFAULT_MIN, PWM_DEFAULT_MIN, PWM_DEFAULT_MIN, PWM_DEFAULT_MIN, PWM_DEFAULT_MIN, PWM_DEFAULT_MIN };

/**
 * PAGE 107
 *
 * maximum PWM values when armed  解锁后最大的pwm值
 *
 */
uint16_t		r_page_servo_control_max[PX4IO_SERVO_COUNT] = { PWM_DEFAULT_MAX, PWM_DEFAULT_MAX, PWM_DEFAULT_MAX, PWM_DEFAULT_MAX, PWM_DEFAULT_MAX, PWM_DEFAULT_MAX, PWM_DEFAULT_MAX, PWM_DEFAULT_MAX };

/**
 * PAGE 108
 *
 * disarmed PWM values for difficult ESCs 对于不同的电调对应解锁
 *
 */
uint16_t		r_page_servo_disarmed[PX4IO_SERVO_COUNT] = { 0, 0, 0, 0, 0, 0, 0, 0 };

int
registers_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)  //寄存器设置，最后一个为包里包含的值的数量
{

	switch (page) { //页选项

	/* handle bulk controls input */ //处理大块控制输入
	case PX4IO_PAGE_CONTROLS:  //来自fmu的控制值

		/* copy channel data */  //复制通道数据
		while ((offset < PX4IO_CONTROL_GROUPS * PX4IO_CONTROL_CHANNELS) && (num_values > 0)) {  //偏移小于数组大小，值数量 ，4个控制组，8个通道

			/* XXX range-check value? */
			r_page_controls[offset] = *values;  //依次填充

			offset++; //偏移自增
			num_values--;  //数量递减
			values++;  //值指针
		}

		system_state.fmu_data_received_time = hrt_absolute_time();  //获取fmu数据时间戳
		r_status_flags |= PX4IO_P_STATUS_FLAGS_FMU_OK;   //fmu ok标志
		r_status_flags &= ~PX4IO_P_STATUS_FLAGS_RAW_PWM; //清除原始pwm值

		break;

	/* handle raw PWM input */   //处理原始pwm输入  伺服pwm值
	case PX4IO_PAGE_DIRECT_PWM:

		/* copy channel data */  //复制通道数据
		while ((offset < PX4IO_CONTROL_CHANNELS) && (num_values > 0)) {  //小于输出通道

			/* XXX range-check value? */
			if (*values != PWM_IGNORE_THIS_CHANNEL) {//忽略0xfff
				r_page_servos[offset] = *values;  //pwm不能到达百分之100，顺序存入8个通道的pwm值
			}

			offset++;
			num_values--;
			values++;
		}

		system_state.fmu_data_received_time = hrt_absolute_time();  //fmu数据接收时间戳
		r_status_flags |= PX4IO_P_STATUS_FLAGS_FMU_OK | PX4IO_P_STATUS_FLAGS_RAW_PWM;  //fmu ok，原始pwm有输入

		break;

	/* handle setup for servo failsafe values */
	case PX4IO_PAGE_FAILSAFE_PWM:  //处理设置伺服失效保护值，失效时各个马达的pwm值

		/* copy channel data */
		while ((offset < PX4IO_SERVO_COUNT) && (num_values > 0)) {  //小于伺服数量

			if (*values == 0) {  //忽略0，pwm忽略0输出
				/* ignore 0 */
			} else if (*values < PWM_LOWEST_MIN) { //限制失效保护pwm值范围
				r_page_servo_failsafe[offset] = PWM_LOWEST_MIN;

			} else if (*values > PWM_HIGHEST_MAX) {
				r_page_servo_failsafe[offset] = PWM_HIGHEST_MAX;

			} else {  //如果在范围内
				r_page_servo_failsafe[offset] = *values;
			}

			/* flag the failsafe values as custom */
			r_setup_arming |= PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM;  //已经定制失效保护pwm

			offset++;
			num_values--;
			values++;
		}

		break;

	case PX4IO_PAGE_CONTROL_MIN_PWM:  //控制的最小pwm值

		/* copy channel data */
		while ((offset < PX4IO_SERVO_COUNT) && (num_values > 0)) { //小于伺服数量

			if (*values == 0) {
				/* ignore 0 */
			} else if (*values > PWM_HIGHEST_MIN) {  //限制到范围内
				r_page_servo_control_min[offset] = PWM_HIGHEST_MIN;

			} else if (*values < PWM_LOWEST_MIN) {
				r_page_servo_control_min[offset] = PWM_LOWEST_MIN;

			} else { //在范围内
				r_page_servo_control_min[offset] = *values;
			}

			offset++;
			num_values--;
			values++;
		}

		break;

	case PX4IO_PAGE_CONTROL_MAX_PWM:  //控制的最大pwm值

		/* copy channel data */
		while ((offset < PX4IO_SERVO_COUNT) && (num_values > 0)) {  //小于伺服数量

			if (*values == 0) {  //忽略百分之0
				/* ignore 0 */
			} else if (*values > PWM_HIGHEST_MAX) { //限制范围内
				r_page_servo_control_max[offset] = PWM_HIGHEST_MAX;

			} else if (*values < PWM_LOWEST_MAX) {
				r_page_servo_control_max[offset] = PWM_LOWEST_MAX;

			} else {
				r_page_servo_control_max[offset] = *values;
			}

			offset++;
			num_values--;
			values++;
		}

		break;

	case PX4IO_PAGE_DISARMED_PWM: {  //设置加锁时pwm值
			/* flag for all outputs */
			bool all_disarmed_off = true;  //所有加锁关闭

			/* copy channel data */
			while ((offset < PX4IO_SERVO_COUNT) && (num_values > 0)) {  //小于伺服数量

				if (*values == 0) {  //值为0才加锁成功
					/* 0 means disabling always PWM */
					r_page_servo_disarmed[offset] = 0;  //如果为0，一直禁止pwm

				} else if (*values < PWM_LOWEST_MIN) { //限制到范围内
					r_page_servo_disarmed[offset] = PWM_LOWEST_MIN;
					all_disarmed_off = false;  //所有加锁没有关闭

				} else if (*values > PWM_HIGHEST_MAX) {
					r_page_servo_disarmed[offset] = PWM_HIGHEST_MAX;
					all_disarmed_off = false;

				} else {
					r_page_servo_disarmed[offset] = *values;
					all_disarmed_off = false;
				}

				offset++;
				num_values--;
				values++;
			}

			if (all_disarmed_off) {  //如果所有的加锁关闭成功
				/* disable PWM output if disarmed */
				r_setup_arming &= ~(PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE);  //如果加锁成功，一直解锁所有pwm输出标志清零

			} else {
				/* enable PWM output always */
				r_setup_arming |= PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE;  //一直使能pwm，一直解锁pwm
			}
		}
		break;

	/* handle text going to the mixer parser */  //处理文本，到混控器解析
	case PX4IO_PAGE_MIXERLOAD:
		/* do not change the mixer if FMU is armed and IO's safety is off  如果fmu解锁并且io安全开关关闭，不要改变混控器，
		 * this state defines an active system. This check is done in the 因为这个状态定义了一个活动的系统。在文本处理功能函数中，这个检查已经完成
		 * text handling function.
		 */
		return mixer_handle_text(values, num_values * sizeof(*values));  //处理混控文本解析,长度转换为字节

	default:

		/* avoid offset wrap */ //避免偏移掩护
		if ((offset + num_values) > 255) { //如果偏移加上包中值的数量大于255
			num_values = 255 - offset;
		}

		/* iterate individual registers, set each in turn */ //迭代个别的寄存器，挨个进行设置
		while (num_values--) { //挨个轮询
			if (registers_set_one(page, offset, *values)) { //页和偏移不变，设置一些标志为。one表示单个位
				return -1;
			}

			offset++;
			values++;
		}

		break;
	}

	return 0;
}

static int
registers_set_one(uint8_t page, uint8_t offset, uint16_t value)  //设置一个寄存器
{
	switch (page) {  //页面

	case PX4IO_PAGE_STATUS:  //页面状态
		switch (offset) { //偏移
		case PX4IO_P_STATUS_ALARMS:  //解锁
			/* clear bits being written */
			r_status_alarms &= ~value;  //解锁状态
			break;

		case PX4IO_P_STATUS_FLAGS:

			/*
			 * Allow FMU override of arming state (to allow in-air restores),
			 * but only if the arming state is not in sync on the IO side.
			 * 允许fmu覆盖解锁状态，允许控制保存
			 */

			if (PX4IO_P_STATUS_FLAGS_MIXER_OK & value) { //混控器ok
				r_status_flags |= PX4IO_P_STATUS_FLAGS_MIXER_OK;

			} else if (!(r_status_flags & PX4IO_P_STATUS_FLAGS_ARM_SYNC)) { //同步解锁fmu和io
				r_status_flags = value;

			}

			if (PX4IO_P_STATUS_FLAGS_MIXER_OK & r_status_flags) { //如果混控ok

				/* update failsafe values, now that the mixer is set to ok */
				mixer_set_failsafe(); //更新失效保护值
			}

			break;

		default:
			/* just ignore writes to other registers in this page */
			break;
		}

		break;

	case PX4IO_PAGE_SETUP:  //页设置
		switch (offset) {
		case PX4IO_P_SETUP_FEATURES:

			value &= PX4IO_P_SETUP_FEATURES_VALID;  //信号性能值

			/* some of the options conflict - give S.BUS out precedence, then ADC RSSI, then PWM RSSI */

			/* switch S.Bus output pin as needed */  //开关sbus输出，如果有需要
#ifdef ENABLE_SBUS_OUT
			ENABLE_SBUS_OUT(value & (PX4IO_P_SETUP_FEATURES_SBUS1_OUT | PX4IO_P_SETUP_FEATURES_SBUS2_OUT));

			/* disable the conflicting options with SBUS 1 */
			if (value & (PX4IO_P_SETUP_FEATURES_SBUS1_OUT)) {
				value &= ~(PX4IO_P_SETUP_FEATURES_PWM_RSSI |
					   PX4IO_P_SETUP_FEATURES_ADC_RSSI |
					   PX4IO_P_SETUP_FEATURES_SBUS2_OUT);
			}

			/* disable the conflicting options with SBUS 2 */
			if (value & (PX4IO_P_SETUP_FEATURES_SBUS2_OUT)) {
				value &= ~(PX4IO_P_SETUP_FEATURES_PWM_RSSI |
					   PX4IO_P_SETUP_FEATURES_ADC_RSSI |
					   PX4IO_P_SETUP_FEATURES_SBUS1_OUT);
			}

#endif

			/* disable the conflicting options with ADC RSSI */
			if (value & (PX4IO_P_SETUP_FEATURES_ADC_RSSI)) {
				value &= ~(PX4IO_P_SETUP_FEATURES_PWM_RSSI |
					   PX4IO_P_SETUP_FEATURES_SBUS1_OUT |
					   PX4IO_P_SETUP_FEATURES_SBUS2_OUT);
			}

			/* disable the conflicting options with PWM RSSI (without effect here, but for completeness) */
			if (value & (PX4IO_P_SETUP_FEATURES_PWM_RSSI)) {
				value &= ~(PX4IO_P_SETUP_FEATURES_ADC_RSSI |
					   PX4IO_P_SETUP_FEATURES_SBUS1_OUT |
					   PX4IO_P_SETUP_FEATURES_SBUS2_OUT);
			}

			/* apply changes */
			r_setup_features = value;

			break;

		case PX4IO_P_SETUP_ARMING:  //设置解锁

			value &= PX4IO_P_SETUP_ARMING_VALID;  //设置解锁有效id

			/*
			 * Update arming state - disarm if no longer OK.  更新解锁状态，如果长时间没ok，加锁
			 * This builds on the requirement that the FMU driver
			 * asks about the FMU arming state on initialization,
			 * so that an in-air reset of FMU can not lead to a
			 * lockup of the IO arming state.
			 */

			if (value & PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED) {  //设置解锁，rc处理禁止
				r_status_flags |= PX4IO_P_STATUS_FLAGS_INIT_OK;  //io初始化成功，没错误
			}

			/*
			 * If the failsafe termination flag is set, do not allow the autopilot to unset it
			 */
			value |= (r_setup_arming & PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE);

			/*
			 * If failsafe termination is enabled and force failsafe bit is set, do not allow
			 * the autopilot to clear it.
			 */
			if (r_setup_arming & PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE) {
				value |= (r_setup_arming & PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE);
			}

			r_setup_arming = value;

			break;

		case PX4IO_P_SETUP_PWM_RATES:  //设置pwm速率
			value &= PX4IO_P_SETUP_RATES_VALID;
			pwm_configure_rates(value, r_setup_pwm_defaultrate, r_setup_pwm_altrate);
			break;

		case PX4IO_P_SETUP_PWM_DEFAULTRATE: //设置默认的pwm速率
			if (value < 25) {
				value = 25;
			}

			if (value > 400) {
				value = 400;
			}

			pwm_configure_rates(r_setup_pwm_rates, value, r_setup_pwm_altrate);
			break;

		case PX4IO_P_SETUP_PWM_ALTRATE: //设置交替pwm速率
			if (value < 25) {
				value = 25;
			}

			if (value > 400) {
				value = 400;
			}

			pwm_configure_rates(r_setup_pwm_rates, r_setup_pwm_defaultrate, value);
			break;

		case PX4IO_P_SETUP_VBATT_SCALE:  //设置vbatt缩放因子
			r_page_setup[PX4IO_P_SETUP_VBATT_SCALE] = value;
			break;

		case PX4IO_P_SETUP_SET_DEBUG:  //设置debug
			r_page_setup[PX4IO_P_SETUP_SET_DEBUG] = value;
			isr_debug(0, "set debug %u\n", (unsigned)r_page_setup[PX4IO_P_SETUP_SET_DEBUG]);
			break;

		case PX4IO_P_SETUP_REBOOT_BL:  //设置复位到bootloader
			if (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) {
				// don't allow reboot while armed  在解锁状态不允许复位
				break;
			}

			// check the magic value
			if (value != PX4IO_REBOOT_BL_MAGIC) {
				break;
			}

			// we schedule a reboot rather than rebooting  计划一个复位而不是立马复位允许io控制板
			// immediately to allow the IO board to ACK
			// the reboot command
			schedule_reboot(100000); //计划一个复位而不是立即复位，允许io板应答复位命令
			break;

		case PX4IO_P_SETUP_DSM:  //设置dsm
			dsm_bind(value & 0x0f, (value >> 4) & 0xF);
			break;

		case PX4IO_P_SETUP_FORCE_SAFETY_ON:  //强制安全开关打开
			if (value == PX4IO_FORCE_SAFETY_MAGIC) {
				r_status_flags &= ~PX4IO_P_STATUS_FLAGS_SAFETY_OFF;
			}

			break;

		case PX4IO_P_SETUP_FORCE_SAFETY_OFF:  //强制安全开关关闭
			if (value == PX4IO_FORCE_SAFETY_MAGIC) {
				r_status_flags |= PX4IO_P_STATUS_FLAGS_SAFETY_OFF;
			}

			break;

		case PX4IO_P_SETUP_RC_THR_FAILSAFE_US:  //设置失效保护阈值单位us
			if (value > 650 && value < 2350) {
				r_page_setup[PX4IO_P_SETUP_RC_THR_FAILSAFE_US] = value;
			}

			break;

		case PX4IO_P_SETUP_PWM_REVERSE:  //设置pwm反转
			r_page_setup[PX4IO_P_SETUP_PWM_REVERSE] = value;
			break;

		case PX4IO_P_SETUP_TRIM_ROLL:  //修剪
		case PX4IO_P_SETUP_TRIM_PITCH:
		case PX4IO_P_SETUP_TRIM_YAW:
			r_page_setup[offset] = value;
			break;

		default:
			return -1;
		}

		break;

	case PX4IO_PAGE_RC_CONFIG: {  //rc配置

			/**
			 * do not allow a RC config change while safety is off
			 */
			if (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) { //如果安全开关关闭，不允许rc配置
				break;
			}

			unsigned channel = offset / PX4IO_P_RC_CONFIG_STRIDE;  //获取通道号
			unsigned index = offset - channel * PX4IO_P_RC_CONFIG_STRIDE;  //指针
			uint16_t *conf = &r_page_rc_input_config[channel * PX4IO_P_RC_CONFIG_STRIDE];

			if (channel >= PX4IO_RC_INPUT_CHANNELS) {  //如果通道大于最大的rc输入通道，返回错误
				return -1;
			}

			/* disable the channel until we have a chance to sanity-check it */
			conf[PX4IO_P_RC_CONFIG_OPTIONS] &= PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;

			switch (index) {  //索引

			case PX4IO_P_RC_CONFIG_MIN:
			case PX4IO_P_RC_CONFIG_CENTER:
			case PX4IO_P_RC_CONFIG_MAX:
			case PX4IO_P_RC_CONFIG_DEADZONE:
			case PX4IO_P_RC_CONFIG_ASSIGNMENT:
				conf[index] = value;
				break;

			case PX4IO_P_RC_CONFIG_OPTIONS:
				value &= PX4IO_P_RC_CONFIG_OPTIONS_VALID;
				r_status_flags |= PX4IO_P_STATUS_FLAGS_INIT_OK;

				/* clear any existing RC disabled flag */
				r_setup_arming &= ~(PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED);

				/* set all options except the enabled option */
				conf[index] = value & ~PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;

				/* should the channel be enabled? */
				/* this option is normally set last */
				if (value & PX4IO_P_RC_CONFIG_OPTIONS_ENABLED) {
					uint8_t count = 0;
					bool disabled = false;

					/* assert min..center..max ordering */
					if (conf[PX4IO_P_RC_CONFIG_MIN] < 500) {
						count++;
					}

					if (conf[PX4IO_P_RC_CONFIG_MAX] > 2500) {
						count++;
					}

					if (conf[PX4IO_P_RC_CONFIG_CENTER] < conf[PX4IO_P_RC_CONFIG_MIN]) {
						count++;
					}

					if (conf[PX4IO_P_RC_CONFIG_CENTER] > conf[PX4IO_P_RC_CONFIG_MAX]) {
						count++;
					}

					/* assert deadzone is sane */
					if (conf[PX4IO_P_RC_CONFIG_DEADZONE] > 500) {
						count++;
					}

					if (conf[PX4IO_P_RC_CONFIG_ASSIGNMENT] == UINT8_MAX) {
						disabled = true;

					} else if ((conf[PX4IO_P_RC_CONFIG_ASSIGNMENT] >= PX4IO_RC_MAPPED_CONTROL_CHANNELS) &&
						   (conf[PX4IO_P_RC_CONFIG_ASSIGNMENT] != PX4IO_P_RC_CONFIG_ASSIGNMENT_MODESWITCH)) {
						count++;
					}

					/* sanity checks pass, enable channel */
					if (count) {
						isr_debug(0, "ERROR: %d config error(s) for RC%d.\n", count, (channel + 1));
						r_status_flags &= ~PX4IO_P_STATUS_FLAGS_INIT_OK;

					} else if (!disabled) {
						conf[index] |= PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;
					}
				}

				break;
				/* inner switch: case PX4IO_P_RC_CONFIG_OPTIONS */

			}

			break;
			/* case PX4IO_RC_PAGE_CONFIG */
		}

	case PX4IO_PAGE_TEST: //页测试
		switch (offset) {
		case PX4IO_P_TEST_LED:
			LED_AMBER(value & 1);
			break;
		}

		break;

	default:
		return -1;
	}

	return 0;
}

uint8_t last_page;
uint8_t last_offset;

int
registers_get(uint8_t page, uint8_t offset, uint16_t **values, unsigned *num_values)
{
#define SELECT_PAGE(_page_name)							\
	do {									\
		*values = (uint16_t *)&_page_name[0];				\
		*num_values = sizeof(_page_name) / sizeof(_page_name[0]);	\
	} while(0) //最后一个表示值的

	switch (page) {

	/*
	 * Handle pages that are updated dynamically at read time.
	 */
	case PX4IO_PAGE_STATUS:
		/* PX4IO_P_STATUS_FREEMEM */  //空余的ram
		{
			struct mallinfo minfo = mallinfo();
			r_page_status[PX4IO_P_STATUS_FREEMEM] = minfo.fordblks;
		}

		/* XXX PX4IO_P_STATUS_CPULOAD */

		/* PX4IO_P_STATUS_FLAGS maintained externally */

		/* PX4IO_P_STATUS_ALARMS maintained externally */

#ifdef ADC_VBATT
		/* PX4IO_P_STATUS_VBATT */
		{
			/*
			 * Coefficients here derived by measurement of the 5-16V
			 * range on one unit, validated on sample points of another unit
			 *
			 * Data in Tools/tests-host/data folder.
			 *
			 * measured slope = 0.004585267878277 (int: 4585)
			 * nominal theoretic slope: 0.00459340659 (int: 4593)
			 * intercept = 0.016646394188076 (int: 16646)
			 * nominal theoretic intercept: 0.00 (int: 0)
			 *
			 */
			unsigned counts = adc_measure(ADC_VBATT);

			if (counts != 0xffff) {
				unsigned mV = (166460 + (counts * 45934)) / 10000;
				unsigned corrected = (mV * r_page_setup[PX4IO_P_SETUP_VBATT_SCALE]) / 10000;

				r_page_status[PX4IO_P_STATUS_VBATT] = corrected;
			}
		}
#endif
#ifdef ADC_IBATT
		/* PX4IO_P_STATUS_IBATT */
		{
			/*
			  note that we have no idea what sort of
			  current sensor is attached, so we just
			  return the raw 12 bit ADC value and let the
			  FMU sort it out, with user selectable
			  configuration for their sensor
			 */
			unsigned counts = adc_measure(ADC_IBATT);

			if (counts != 0xffff) {
				r_page_status[PX4IO_P_STATUS_IBATT] = counts;
			}
		}
#endif
#ifdef ADC_VSERVO
		/* PX4IO_P_STATUS_VSERVO */
		{
			unsigned counts = adc_measure(ADC_VSERVO);

			if (counts != 0xffff) {
				// use 3:1 scaling on 3.3V ADC input
				unsigned mV = counts * 9900 / 4096;
				r_page_status[PX4IO_P_STATUS_VSERVO] = mV;
			}
		}
#endif
#ifdef ADC_RSSI  //接收信号强度指示器
		/* PX4IO_P_STATUS_VRSSI */
		{
			unsigned counts = adc_measure(ADC_RSSI);

			if (counts != 0xffff) {
				// use 1:1 scaling on 3.3V ADC input
				unsigned mV = counts * 3300 / 4096;
				r_page_status[PX4IO_P_STATUS_VRSSI] = mV;
			}
		}
#endif
		/* XXX PX4IO_P_STATUS_PRSSI */

		SELECT_PAGE(r_page_status);  //最开始的宏定义
		break;

	case PX4IO_PAGE_RAW_ADC_INPUT:  //原始adc输入
		memset(r_page_scratch, 0, sizeof(r_page_scratch));
#ifdef ADC_VBATT
		r_page_scratch[0] = adc_measure(ADC_VBATT);
#endif
#ifdef ADC_IBATT
		r_page_scratch[1] = adc_measure(ADC_IBATT);
#endif

#ifdef ADC_VSERVO
		r_page_scratch[0] = adc_measure(ADC_VSERVO);
#endif
#ifdef ADC_RSSI
		r_page_scratch[1] = adc_measure(ADC_RSSI);
#endif
		SELECT_PAGE(r_page_scratch);
		break;

	case PX4IO_PAGE_PWM_INFO: //pwm信息
		memset(r_page_scratch, 0, sizeof(r_page_scratch));

		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			r_page_scratch[PX4IO_RATE_MAP_BASE + i] = up_pwm_servo_get_rate_group(i);
		}

		SELECT_PAGE(r_page_scratch);
		break;

	/*
	 * Pages that are just a straight read of the register state.
	 */

	/* status pages */  //状态页
	case PX4IO_PAGE_CONFIG:
		SELECT_PAGE(r_page_config);
		break;

	case PX4IO_PAGE_ACTUATORS:
		SELECT_PAGE(r_page_actuators);
		break;

	case PX4IO_PAGE_SERVOS:
		SELECT_PAGE(r_page_servos);
		break;

	case PX4IO_PAGE_RAW_RC_INPUT:
		SELECT_PAGE(r_page_raw_rc_input);
		break;

	case PX4IO_PAGE_RC_INPUT:
		SELECT_PAGE(r_page_rc_input);
		break;

	/* readback of input pages */
	case PX4IO_PAGE_SETUP:
		SELECT_PAGE(r_page_setup);
		break;

	case PX4IO_PAGE_CONTROLS:
		SELECT_PAGE(r_page_controls);
		break;

	case PX4IO_PAGE_RC_CONFIG:
		SELECT_PAGE(r_page_rc_input_config);
		break;

	case PX4IO_PAGE_DIRECT_PWM:
		SELECT_PAGE(r_page_servos);
		break;

	case PX4IO_PAGE_FAILSAFE_PWM:
		SELECT_PAGE(r_page_servo_failsafe);
		break;

	case PX4IO_PAGE_CONTROL_MIN_PWM:
		SELECT_PAGE(r_page_servo_control_min);
		break;

	case PX4IO_PAGE_CONTROL_MAX_PWM:
		SELECT_PAGE(r_page_servo_control_max);
		break;

	case PX4IO_PAGE_DISARMED_PWM:
		SELECT_PAGE(r_page_servo_disarmed);
		break;

	default:
		return -1;
	}

#undef SELECT_PAGE
#undef COPY_PAGE

	last_page = page;
	last_offset = offset;

	/* if the offset is at or beyond the end of the page, we have no data */
	if (offset >= *num_values) {
		return -1;
	}

	/* correct the data pointer and count for the offset */
	*values += offset;
	*num_values -= offset;

	return 0;
}

/*
 * Helper function to handle changes to the PWM rate control registers. 帮组功能，处理改变pwm速率控制寄存器
 */
static void
pwm_configure_rates(uint16_t map, uint16_t defaultrate, uint16_t altrate)
{
	for (unsigned pass = 0; pass < 2; pass++) {
		for (unsigned group = 0; group < PX4IO_SERVO_COUNT; group++) {  //扫描所有伺服

			/* get the channel mask for this rate group */  //获得这个速率组的掩码
			uint32_t mask = up_pwm_servo_get_rate_group(group);

			if (mask == 0) {  //如果掩码为0，继续
				continue;
			}

			/* all channels in the group must be either default or alt-rate */
			uint32_t alt = map & mask;

			if (pass == 0) {
				/* preflight */
				if ((alt != 0) && (alt != mask)) {
					/* not a legal map, bail with an alarm */
					r_status_alarms |= PX4IO_P_STATUS_ALARMS_PWM_ERROR;
					return;
				}

			} else {
				/* set it - errors here are unexpected */
				if (alt != 0) {
					if (up_pwm_servo_set_rate_group_update(group, r_setup_pwm_altrate) != OK) {
						r_status_alarms |= PX4IO_P_STATUS_ALARMS_PWM_ERROR;
					}

				} else {
					if (up_pwm_servo_set_rate_group_update(group, r_setup_pwm_defaultrate) != OK) {
						r_status_alarms |= PX4IO_P_STATUS_ALARMS_PWM_ERROR;
					}
				}
			}
		}
	}

	r_setup_pwm_rates = map;
	r_setup_pwm_defaultrate = defaultrate;
	r_setup_pwm_altrate = altrate;
}
