
#pragma once

#define __STDC_FORMAT_MACROS   //标准格式宏定义
#include <inttypes.h>

/**
 * @file protocol.h
 *
 * PX4IO interface protocol.  io口接口协议
 *
 * Communication is performed via writes to and reads from 16-bit virtual
 * registers organised into pages of 255 registers each.   虚拟寄存器
 *
 * The first two bytes of each write select a page and offset address
 * respectively. Subsequent reads and writes increment the offset within
 * the page.  前两个字节分别是选择页和偏移地址，，紧接着的参数是读取和写入在页面偏移的自增加
 *
 * Some pages are read- or write-only.  一些页面只读或者只写
 *
 * Note that some pages may permit offset values greater than 255, which
 * can only be achieved by long writes. The offset does not wrap.  一些页面允许偏移地址大于255
 *
 * Writes to unimplemented registers are ignored. Reads from unimplemented
 * registers return undefined values.  写入一个没有定义的寄存器将会忽略，从一个没有定义的寄存器返回未知数据
 *
 * As convention, values that would be floating point in other parts of
 * the PX4 system are expressed as signed integer values scaled by 10000,
 * e.g. control values range from -10000..10000.  Use the REG_TO_SIGNED and
 * SIGNED_TO_REG macros to convert between register representation and
 * the signed version, and REG_TO_FLOAT/FLOAT_TO_REG to convert to float.
 *作为惯例
 *
 * Note that the implementation of readable pages prefers registers within
 * readable pages to be densely packed. Page numbers do not need to be
 * packed.
 *
 * Definitions marked [1] are only valid on PX4IOv1 boards. Likewise,
 * [2] denotes definitions specific to the PX4IOv2 board.
 */

/* Per C, this is safe for all 2's complement systems */
#define REG_TO_SIGNED(_reg)	((int16_t)(_reg))     //转换为有符号16位整型
#define SIGNED_TO_REG(_signed)	((uint16_t)(_signed))   //转换为16位无符号整型

#define REG_TO_FLOAT(_reg)	((float)REG_TO_SIGNED(_reg) / 10000.0f)   //整型转换为浮点
#define FLOAT_TO_REG(_float)	SIGNED_TO_REG((int16_t)((_float) * 10000.0f))  //浮点数转换为有符号整型

#define PX4IO_PROTOCOL_VERSION		4   //io协议版本

/* maximum allowable sizes on this protocol version */  //在这个版本中，最大可用的大小
#define PX4IO_PROTOCOL_MAX_CONTROL_COUNT	8	/**< The protocol does not support more than set here, individual units might support less - see PX4IO_P_CONFIG_CONTROL_COUNT */

/* static configuration page */   //静态配置页
#define PX4IO_PAGE_CONFIG		0
#define PX4IO_P_CONFIG_PROTOCOL_VERSION		0	/* PX4IO_PROTOCOL_VERSION */  //协议版本
#define PX4IO_P_CONFIG_HARDWARE_VERSION		1	/* magic numbers TBD */  //硬件版本
#define PX4IO_P_CONFIG_BOOTLOADER_VERSION	2	/* get this how? */  //bootloader版本
#define PX4IO_P_CONFIG_MAX_TRANSFER		3	/* maximum I2C transfer size */  //最大i2c传输大小
#define PX4IO_P_CONFIG_CONTROL_COUNT		4	/* hardcoded max control count supported */  //硬件代码。支持的最大控制数目
#define PX4IO_P_CONFIG_ACTUATOR_COUNT		5	/* hardcoded max actuator output count */    //硬件代码，最大执行输出数目
#define PX4IO_P_CONFIG_RC_INPUT_COUNT		6	/* hardcoded max R/C input count supported */   //硬件代码，最大rc输入数量支持
#define PX4IO_P_CONFIG_ADC_INPUT_COUNT		7	/* hardcoded max ADC inputs */   //硬件代码，最大adc输入
#define PX4IO_P_CONFIG_RELAY_COUNT		8	/* hardcoded # of relay outputs */    //硬件代码。继电器输出
#define PX4IO_P_CONFIG_CONTROL_GROUP_COUNT	8	/**< hardcoded # of control groups*/  //硬件代码，控制组数量

/* dynamic status page */     //动态状态页
#define PX4IO_PAGE_STATUS		1    //页面状态
#define PX4IO_P_STATUS_FREEMEM			0  //空余ram状态
#define PX4IO_P_STATUS_CPULOAD			1  //cpu负载

#define PX4IO_P_STATUS_FLAGS			2	 /* monitoring flags */    //监控标志
#define PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED	(1 << 0) /* arm-ok and locally armed */  //成功解锁，和本地解锁
#define PX4IO_P_STATUS_FLAGS_OVERRIDE		(1 << 1) /* in manual override */     //手动覆盖
#define PX4IO_P_STATUS_FLAGS_RC_OK		(1 << 2) /* RC input is valid */          //rc输入有效
#define PX4IO_P_STATUS_FLAGS_RC_PPM		(1 << 3) /* PPM input is valid */           //ppm输入有效
#define PX4IO_P_STATUS_FLAGS_RC_DSM		(1 << 4) /* DSM input is valid */           //dsm输入有效
#define PX4IO_P_STATUS_FLAGS_RC_SBUS		(1 << 5) /* SBUS input is valid */       //sbus输入有效
#define PX4IO_P_STATUS_FLAGS_FMU_OK		(1 << 6) /* controls from FMU are valid */   //来自fmu输入有效
#define PX4IO_P_STATUS_FLAGS_RAW_PWM		(1 << 7) /* raw PWM from FMU is bypassing the mixer */  //来自fmu的原始pwm绕过混控
#define PX4IO_P_STATUS_FLAGS_MIXER_OK		(1 << 8) /* mixer is OK */    //混控状态ok
#define PX4IO_P_STATUS_FLAGS_ARM_SYNC		(1 << 9) /* the arming state between IO and FMU is in sync */   //在io和fmu解锁状态同步
#define PX4IO_P_STATUS_FLAGS_INIT_OK		(1 << 10) /* initialisation of the IO completed without error */   //初始化io完成，并没有错误
#define PX4IO_P_STATUS_FLAGS_FAILSAFE		(1 << 11) /* failsafe is active */    //失效保护激活
#define PX4IO_P_STATUS_FLAGS_SAFETY_OFF		(1 << 12) /* safety is off */      //安全开关关闭
#define PX4IO_P_STATUS_FLAGS_FMU_INITIALIZED	(1 << 13) /* FMU was initialized and OK once */  //fmu已经初始化
#define PX4IO_P_STATUS_FLAGS_RC_ST24		(1 << 14) /* ST24 input is valid */   //st24输入有效
#define PX4IO_P_STATUS_FLAGS_RC_SUMD		(1 << 15) /* SUMD input is valid */   //sumd输入有效

#define PX4IO_P_STATUS_ALARMS			3	 /* alarm flags - alarms latch, write 1 to a bit to clear it */  //解锁标志，解锁锁定
#define PX4IO_P_STATUS_ALARMS_VBATT_LOW		(1 << 0) /* [1] VBatt is very close to regulator dropout */   //电池电压低
#define PX4IO_P_STATUS_ALARMS_TEMPERATURE	(1 << 1) /* board temperature is high */   //板子温度高
#define PX4IO_P_STATUS_ALARMS_SERVO_CURRENT	(1 << 2) /* [1] servo current limit was exceeded */   //伺服过流了
#define PX4IO_P_STATUS_ALARMS_ACC_CURRENT	(1 << 3) /* [1] accessory current limit was exceeded */  //加速计超过量程
#define PX4IO_P_STATUS_ALARMS_FMU_LOST		(1 << 4) /* timed out waiting for controls from FMU */    //fmu超时失控
#define PX4IO_P_STATUS_ALARMS_RC_LOST		(1 << 5) /* timed out waiting for RC input */         //rc超时失控开关
#define PX4IO_P_STATUS_ALARMS_PWM_ERROR		(1 << 6) /* PWM configuration or output was bad */    //pwm配置或者输出错误
#define PX4IO_P_STATUS_ALARMS_VSERVO_FAULT	(1 << 7) /* [2] VServo was out of the valid range (2.5 - 5.5 V) */  //servo输出电压不在有效范围内

#define PX4IO_P_STATUS_VBATT			4	/* [1] battery voltage in mV */   //电池电压
#define PX4IO_P_STATUS_IBATT			5	/* [1] battery current (raw ADC) */  //电池电流
#define PX4IO_P_STATUS_VSERVO			6	/* [2] servo rail voltage in mV */  //伺服电压
#define PX4IO_P_STATUS_VRSSI			7	/* [2] RSSI voltage */   //信号质量电压
#define PX4IO_P_STATUS_PRSSI			8	/* [2] RSSI PWM value */  //信号质量pwm值

#define PX4IO_P_STATUS_MIXER			9	 /* mixer actuator limit flags */   //混控执行限制标志
#define PX4IO_P_STATUS_MIXER_LOWER_LIMIT 		(1 << 0) /**< at least one actuator output has reached lower limit */  //至少有一个执行机构输出到达最低限制
#define PX4IO_P_STATUS_MIXER_UPPER_LIMIT 		(1 << 1) /**< at least one actuator output has reached upper limit */  //至少有一个执行机构输出达到最高限制
#define PX4IO_P_STATUS_MIXER_YAW_LIMIT 			(1 << 2) /**< yaw control is limited because it causes output clipping */  //偏航轴控制被限制，因为它倒置输出恰好

/* array of post-mix actuator outputs, -10000..10000 */  //发布混合执行输出数组
#define PX4IO_PAGE_ACTUATORS		2		/* 0..CONFIG_ACTUATOR_COUNT-1 */

/* array of PWM servo output values, microseconds */  //pwm伺服初始值数组，单位us
#define PX4IO_PAGE_SERVOS		3		/* 0..CONFIG_ACTUATOR_COUNT-1 */

/* array of raw RC input values, microseconds */  //原始rc输入值数组，单位微秒
#define PX4IO_PAGE_RAW_RC_INPUT		4
#define PX4IO_P_RAW_RC_COUNT			0	/* number of valid channels */  //有效通道数目
#define PX4IO_P_RAW_RC_FLAGS			1	/* RC detail status flags */    //rc状态标志
#define PX4IO_P_RAW_RC_FLAGS_FRAME_DROP		(1 << 0) /* single frame drop */
#define PX4IO_P_RAW_RC_FLAGS_FAILSAFE		(1 << 1) /* receiver is in failsafe mode */  //接收机处于失效保护模式
#define PX4IO_P_RAW_RC_FLAGS_RC_DSM11		(1 << 2) /* DSM decoding is 11 bit mode */
#define PX4IO_P_RAW_RC_FLAGS_MAPPING_OK		(1 << 3) /* Channel mapping is ok */
#define PX4IO_P_RAW_RC_FLAGS_RC_OK		(1 << 4) /* RC reception ok */  //rc接收有效

#define PX4IO_P_RAW_RC_NRSSI			2	/* [2] Normalized RSSI value, 0: no reception, 255: perfect reception */
#define PX4IO_P_RAW_RC_DATA			3	/* [1] + [2] Details about the RC source (PPM frame length, Spektrum protocol type) */
#define PX4IO_P_RAW_FRAME_COUNT			4	/* Number of total received frames (wrapping counter) */
#define PX4IO_P_RAW_LOST_FRAME_COUNT		5	/* Number of total dropped frames (wrapping counter) */
#define PX4IO_P_RAW_RC_BASE			6	/* CONFIG_RC_INPUT_COUNT channels from here */

/* array of scaled RC input values, -10000..10000 */  //rc输入缩放值数组
#define PX4IO_PAGE_RC_INPUT		5
#define PX4IO_P_RC_VALID			0	/* bitmask of valid controls */
#define PX4IO_P_RC_BASE				1	/* CONFIG_RC_INPUT_COUNT controls from here */

/* array of raw ADC values */  //原始adc值数组
#define PX4IO_PAGE_RAW_ADC_INPUT	6		/* 0..CONFIG_ADC_INPUT_COUNT-1 */

/* PWM servo information */  //pwm伺服信息
#define PX4IO_PAGE_PWM_INFO		7
#define PX4IO_RATE_MAP_BASE			0	/* 0..CONFIG_ACTUATOR_COUNT bitmaps of PWM rate groups */

/* setup page */  //设置页面
#define PX4IO_PAGE_SETUP		50
#define PX4IO_P_SETUP_FEATURES			0
#define PX4IO_P_SETUP_FEATURES_SBUS1_OUT	(1 << 0) /**< enable S.Bus v1 output */
#define PX4IO_P_SETUP_FEATURES_SBUS2_OUT	(1 << 1) /**< enable S.Bus v2 output */
#define PX4IO_P_SETUP_FEATURES_PWM_RSSI		(1 << 2) /**< enable PWM RSSI parsing */  //使能pwm信号质量分析
#define PX4IO_P_SETUP_FEATURES_ADC_RSSI		(1 << 3) /**< enable ADC RSSI parsing */   //使能adc信号质量分析

#define PX4IO_P_SETUP_ARMING			1	 /* arming controls */  //设置解锁控制
#define PX4IO_P_SETUP_ARMING_IO_ARM_OK		(1 << 0) /* OK to arm the IO side */   //io部分解锁成功
#define PX4IO_P_SETUP_ARMING_FMU_ARMED		(1 << 1) /* FMU is already armed */   //fmu已经解锁
#define PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK	(1 << 2) /* OK to switch to manual override via override RC channel */  //从经过覆盖rc通道转换到手动覆盖
#define PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM	(1 << 3) /* use custom failsafe values, not 0 values of mixer */
#define PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK	(1 << 4) /* OK to try in-air restart */  //尝试在空中复位
#define PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE	(1 << 5) /* Output of PWM right after startup enabled to help ESCs initialize and prevent them from beeping */
#define PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED	(1 << 6) /* Disable the IO-internal evaluation of the RC */  //禁止io内部估算rc
#define PX4IO_P_SETUP_ARMING_LOCKDOWN		(1 << 7) /* If set, the system operates normally, but won't actuate any servos */
#define PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE	(1 << 8) /* If set, the system will always output the failsafe values */  //系统将会用于处于失效保护
#define PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE	(1 << 9) /* If set, the system will never return from a failsafe, but remain in failsafe once triggered. */
#define PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE	(1 << 10) /* If set then on FMU failure override is immediate. Othewise it waits for the mode switch to go past the override thrshold */

#define PX4IO_P_SETUP_PWM_RATES			2	/* bitmask, 0 = low rate, 1 = high rate */  //pwm速率
#define PX4IO_P_SETUP_PWM_DEFAULTRATE		3	/* 'low' PWM frame output rate in Hz */  //pwm默认低速率
#define PX4IO_P_SETUP_PWM_ALTRATE		4	/* 'high' PWM frame output rate in Hz */  //pwm默认高速率

#define PX4IO_P_SETUP_RELAYS_PAD		5

#define PX4IO_P_SETUP_VBATT_SCALE		6	/* hardware rev [1] battery voltage correction factor (float) */  //电池电压修正因数
#define PX4IO_P_SETUP_VSERVO_SCALE		6	/* hardware rev [2] servo voltage correction factor (float) */  //伺服电压修正因数
#define PX4IO_P_SETUP_DSM			7	/* DSM bind state */  //dsm绑定状态
enum {							/* DSM bind states */
	dsm_bind_power_down = 0,
	dsm_bind_power_up,
	dsm_bind_set_rx_out,
	dsm_bind_send_pulses,
	dsm_bind_reinit_uart
};
/* 8 */
#define PX4IO_P_SETUP_SET_DEBUG			9	/* debug level for IO board */  //io控制板的debug等级

#define PX4IO_P_SETUP_REBOOT_BL			10	/* reboot IO into bootloader */  //复位io到bootloader
#define PX4IO_REBOOT_BL_MAGIC			14662	/* required argument for reboot (random) */

#define PX4IO_P_SETUP_CRC			11	/* get CRC of IO firmware */  //获得io固件的crc值
/* storage space of 12 occupied by CRC */
#define PX4IO_P_SETUP_FORCE_SAFETY_OFF		12	/* force safety switch into
                                                           'armed' (PWM enabled) state - this is a non-data write and
                                                           hence index 12 can safely be used. */
#define PX4IO_P_SETUP_RC_THR_FAILSAFE_US	13	/**< the throttle failsafe pulse length in microseconds */  //失效保护脉冲长度 单位us

#define PX4IO_P_SETUP_FORCE_SAFETY_ON		14	/* force safety switch into 'disarmed' (PWM disabled state) */  //强制安全开关加锁
#define PX4IO_FORCE_SAFETY_MAGIC		22027	/* required argument for force safety (random) */  //强制安全，需要参数

#define PX4IO_P_SETUP_PWM_REVERSE		15	/**< Bitmask to reverse PWM channels 1-8 */  //接收pwm通道的位掩码
#define PX4IO_P_SETUP_TRIM_ROLL			16	/**< Roll trim, in actuator units */  //执行单元中，横滚轴修剪
#define PX4IO_P_SETUP_TRIM_PITCH		17	/**< Pitch trim, in actuator units */
#define PX4IO_P_SETUP_TRIM_YAW			18	/**< Yaw trim, in actuator units */

/* autopilot control values, -10000..10000 */  //自驾仪控制值
#define PX4IO_PAGE_CONTROLS			51	/**< actuator control groups, one after the other, 8 wide */  //执行机构控制组
#define PX4IO_P_CONTROLS_GROUP_0		(PX4IO_PROTOCOL_MAX_CONTROL_COUNT * 0)	/**< 0..PX4IO_PROTOCOL_MAX_CONTROL_COUNT - 1 */
#define PX4IO_P_CONTROLS_GROUP_1		(PX4IO_PROTOCOL_MAX_CONTROL_COUNT * 1)	/**< 0..PX4IO_PROTOCOL_MAX_CONTROL_COUNT - 1 */
#define PX4IO_P_CONTROLS_GROUP_2		(PX4IO_PROTOCOL_MAX_CONTROL_COUNT * 2)	/**< 0..PX4IO_PROTOCOL_MAX_CONTROL_COUNT - 1 */
#define PX4IO_P_CONTROLS_GROUP_3		(PX4IO_PROTOCOL_MAX_CONTROL_COUNT * 3)	/**< 0..PX4IO_PROTOCOL_MAX_CONTROL_COUNT - 1 */

#define PX4IO_P_CONTROLS_GROUP_VALID		64  //控制组有效标志
#define PX4IO_P_CONTROLS_GROUP_VALID_GROUP0	(1 << 0) /**< group 0 is valid / received */  //控制组0有效/接收到数据
#define PX4IO_P_CONTROLS_GROUP_VALID_GROUP1	(1 << 1) /**< group 1 is valid / received */
#define PX4IO_P_CONTROLS_GROUP_VALID_GROUP2	(1 << 2) /**< group 2 is valid / received */
#define PX4IO_P_CONTROLS_GROUP_VALID_GROUP3	(1 << 3) /**< group 3 is valid / received */

/* raw text load to the mixer parser - ignores offset */  //原始文本载入混控器解释，忽略偏移
#define PX4IO_PAGE_MIXERLOAD			52

/* R/C channel config */  //rc通道配置
#define PX4IO_PAGE_RC_CONFIG			53		/**< R/C input configuration */  //rc输入配置
#define PX4IO_P_RC_CONFIG_MIN			0		/**< lowest input value */  //最低输入有效值
#define PX4IO_P_RC_CONFIG_CENTER		1		/**< center input value */   //中间输入有效值
#define PX4IO_P_RC_CONFIG_MAX			2		/**< highest input value */   //最大输入有效值
#define PX4IO_P_RC_CONFIG_DEADZONE		3		/**< band around center that is ignored */  //死区
#define PX4IO_P_RC_CONFIG_ASSIGNMENT		4		/**< mapped input value */  //输入有效值映射
#define PX4IO_P_RC_CONFIG_ASSIGNMENT_MODESWITCH	100		/**< magic value for mode switch */
#define PX4IO_P_RC_CONFIG_OPTIONS		5		/**< channel options bitmask */  //通道选择掩码
#define PX4IO_P_RC_CONFIG_OPTIONS_ENABLED	(1 << 0)  //通道选项使能
#define PX4IO_P_RC_CONFIG_OPTIONS_REVERSE	(1 << 1)  //通道选项反转
#define PX4IO_P_RC_CONFIG_STRIDE		6		/**< spacing between channel config data */

/* PWM output - overrides mixer */
#define PX4IO_PAGE_DIRECT_PWM			54		/**< 0..CONFIG_ACTUATOR_COUNT-1 */

/* PWM failsafe values - zero disables the output */   //pwm失效保护值，0表示禁止输出
#define PX4IO_PAGE_FAILSAFE_PWM			55		/**< 0..CONFIG_ACTUATOR_COUNT-1 */

/* PWM failsafe values - zero disables the output */
#define PX4IO_PAGE_SENSORS			56		/**< Sensors connected to PX4IO */  //连接在io口上的传感器
#define PX4IO_P_SENSORS_ALTITUDE		0		/**< Altitude of an external sensor (HoTT or S.BUS2) */

/* Debug and test page - not used in normal operation */
#define PX4IO_PAGE_TEST				127   //页测试，不在正常操作中使用
#define PX4IO_P_TEST_LED			0		/**< set the amber LED on/off */ //设置琥珀色灯打开和关闭

/* PWM minimum values for certain ESCs */  //pwm最小值对于特定的电调
#define PX4IO_PAGE_CONTROL_MIN_PWM		106		/**< 0..CONFIG_ACTUATOR_COUNT-1 */

/* PWM maximum values for certain ESCs */   //pwm最大值堆与特定电调
#define PX4IO_PAGE_CONTROL_MAX_PWM		107		/**< 0..CONFIG_ACTUATOR_COUNT-1 */

/* PWM disarmed values that are active, even when SAFETY_SAFE */  //pwm加锁值
#define PX4IO_PAGE_DISARMED_PWM		108			/* 0..CONFIG_ACTUATOR_COUNT-1 */

/**
 * As-needed mixer data upload.
 *
 * This message adds text to the mixer text buffer; the text
 * buffer is drained as the definitions are consumed.
 */
#pragma pack(push, 1)
struct px4io_mixdata {
	uint16_t	f2i_mixer_magic;
#define F2I_MIXER_MAGIC		0x6d74

	uint8_t		action;
#define F2I_MIXER_ACTION_RESET			0
#define F2I_MIXER_ACTION_APPEND			1

	char		text[0];	/* actual text size may vary */
};
#pragma pack(pop)

/**
 * Serial protocol encapsulation.  串口协议封装
 */

#define PKT_MAX_REGS	32 // by agreement w/FMU  最大寄存器数量

#pragma pack(push, 1)
struct IOPacket {
	uint8_t 	count_code;  //
	uint8_t 	crc;  //校验值
	uint8_t 	page;  //页面
	uint8_t 	offset;  //偏移地址
	uint16_t	regs[PKT_MAX_REGS];  //寄存器
};
#pragma pack(pop)

#define PKT_CODE_READ		0x00	/* FMU->IO read transaction */  //fmu读取io口
#define PKT_CODE_WRITE		0x40	/* FMU->IO write transaction */  //fmu写入io口
#define PKT_CODE_SUCCESS	0x00	/* IO->FMU success reply */   //io响应fmu应答
#define PKT_CODE_CORRUPT	0x40	/* IO->FMU bad packet reply */    //io响应错误包应答
#define PKT_CODE_ERROR		0x80	/* IO->FMU register op error reply */  //io响应寄存器操作错误应答

#define PKT_CODE_MASK		0xc0  //代码掩码  1100
#define PKT_COUNT_MASK		0x3f   //代码数量掩码 0011 1111

#define PKT_COUNT(_p)	((_p).count_code & PKT_COUNT_MASK)  //包里包含的值的数量
#define PKT_CODE(_p)	((_p).count_code & PKT_CODE_MASK)   //解析代码
#define PKT_SIZE(_p)	((size_t)((uint8_t *)&((_p).regs[PKT_COUNT(_p)]) - ((uint8_t *)&(_p))))  //解析包大小

static const uint8_t crc8_tab[256] __attribute__((unused)) = {
	0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
	0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
	0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
	0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
	0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
	0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
	0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
	0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
	0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
	0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
	0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
	0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
	0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
	0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
	0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
	0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
	0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
	0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
	0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
	0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
	0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
	0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
	0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
	0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
	0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
	0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
	0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
	0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
	0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
	0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
	0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
	0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

static uint8_t crc_packet(struct IOPacket *pkt) __attribute__((unused));
static uint8_t
crc_packet(struct IOPacket *pkt)
{
	uint8_t *end = (uint8_t *)(&pkt->regs[PKT_COUNT(*pkt)]);
	uint8_t *p = (uint8_t *)pkt;
	uint8_t c = 0;

	while (p < end) {
		c = crc8_tab[c ^ * (p++)];
	}

	return c;
}
