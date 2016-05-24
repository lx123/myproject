/**
 * @file px4io.cpp
 * Driver for the PX4IO board.
 *
 * PX4IO is connected via I2C or DMA enabled high-speed UART.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <crc32.h>

#include <arch/board/board.h>

#include <drivers/device/device_nuttx.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_sbus.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>

#include <systemlib/mixer/mixer.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/param/param.h>
#include <systemlib/circuit_breaker.h>

#include <uORB/topics/actuator_controls.h>   //执行器控制，和四个控制组
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_controls_1.h>
#include <uORB/topics/actuator_controls_2.h>
#include <uORB/topics/actuator_controls_3.h>
#include <uORB/topics/actuator_outputs.h>  //执行机构输出
#include <uORB/topics/actuator_armed.h>  //执行结构解锁
#include <uORB/topics/safety.h>   //安全开关
#include <uORB/topics/vehicle_control_mode.h>  //飞行器控制模式
#include <uORB/topics/vehicle_command.h>    //飞行器命令
#include <uORB/topics/rc_channels.h>    //rc通道
#include <uORB/topics/battery_status.h>   //电池状态
#include <uORB/topics/servorail_status.h>
#include <uORB/topics/parameter_update.h>   //参数更新
#include <uORB/topics/multirotor_motor_limits.h>  //多旋翼马达限制

#include <debug.h>

#include <mavlink/mavlink_log.h>   //mavlink日志
#include <modules/iofirmware/protocol.h>  //固件协议

#include "uploader.h"

#include "modules/dataman/dataman.h"

#include "io_driver.h"

#define PX4IO_SET_DEBUG			_IOC(0xff00, 0)
#define PX4IO_INAIR_RESTART_ENABLE	_IOC(0xff00, 1)  //空中复位使能
#define PX4IO_REBOOT_BOOTLOADER		_IOC(0xff00, 2)  //复位到bootloader
#define PX4IO_CHECK_CRC			_IOC(0xff00, 3)   //检查io控制板的crc

#define UPDATE_INTERVAL_MIN		2			// 2 ms	-> 500 Hz  //最小更新间隔
#define ORB_CHECK_INTERVAL		200000		// 200 ms -> 5 Hz   //orb检查间隔
#define IO_POLL_INTERVAL		20000		// 20 ms -> 50 Hz  //io轮询间隔

/**
 * The PX4IO class.
 *
 * Encapsulates PX4FMU to PX4IO communications modeled as file operations. 压缩通信作为一个文件操作
 */
class PX4IO : public device::CDev //继承于字符设备
{
public:
	/**
	 * Constructor.
	 *
	 * Initialize all class variables.
	 */
	PX4IO(device::Device *interface);

	/**
	 * Destructor.
	 *
	 * Wait for worker thread to terminate.
	 */
	virtual ~PX4IO();

	/**
	 * Initialize the PX4IO class.
	 *
	 * Retrieve relevant initial system parameters. Initialize PX4IO registers.
	 * 检索相关初始化系统参数，初始化io寄存器
	 */
	virtual int		init();

	/**
	 * Initialize the PX4IO class.  初始化io类
	 *
	 * Retrieve relevant initial system parameters. Initialize PX4IO registers.
	 *
	 * @param disable_rc_handling set to true to forbid override / RC handling on IO
	 * 如果输入为true，禁止覆盖/rc处理，在io中
	 */
	int			init(bool disable_rc_handling);  //函数的重载

	/**
	 * Detect if a PX4IO is connected.  检测io口是否被连接
	 *
	 * Only validate if there is a PX4IO to talk to.
	 */
    virtual int		detect();  //检测io口是否连接上

	/**
	 * IO Control handler.
	 *
	 * Handle all IOCTL calls to the PX4IO file descriptor.
	 *
	 * @param[in] filp file handle (not used). This function is always called directly through object reference
	 * @param[in] cmd the IOCTL command
	 * @param[in] the IOCTL command parameter (optional)
	 */
	virtual int		ioctl(file *filp, int cmd, unsigned long arg);

	/**
	 * write handler.  写入处理
	 *
	 * Handle writes to the PX4IO file descriptor.  处理写入io文件描述符
	 *
	 * @param[in] filp file handle (not used). This function is always called directly through object reference
	 * @param[in] buffer pointer to the data buffer to be written
	 * @param[in] len size in bytes to be written
	 * @return number of bytes written
	 */
	virtual ssize_t		write(file *filp, const char *buffer, size_t len);

	/**
	* Set the update rate for actuator outputs from FMU to IO.  执行输出更新速率从fmu到io
	*
	* @param[in] rate		The rate in Hz actuator outpus are sent to IO.
	* 			Min 10 Hz, max 400 Hz
	*/
	int      		set_update_rate(int rate);  //速率范围10-400Hz

	/**
	* Set the battery current scaling and bias  设置电池电流缩放和偏差
	*
	* @param[in] amp_per_volt
	* @param[in] amp_bias
	*/
	void      		set_battery_current_scaling(float amp_per_volt, float amp_bias);

	/**
	 * Push failsafe values to IO.   推送失效保护值到io控制板
	 *
	 * @param[in] vals	Failsafe control inputs: in us PPM (900 for zero, 1500 for centered, 2100 for full)
	 * @param[in] len	Number of channels, could up to 8
	 */
	int			set_failsafe_values(const uint16_t *vals, unsigned len);

	/**
	 * Disable RC input handling  禁止rc输入处理
	 */
	int			disable_rc_handling();

	/**
	 * Print IO status.  打印io状态
	 *
	 * Print all relevant IO status information  打印所有相关的io状态信息
	 *
	 * @param extended_status Shows more verbose information (in particular RC config)
	 */
	void			print_status(bool extended_status);

	/**
	 * Fetch and print debug console output.  获取和打印debug扣扣你告知输出
	 */
	int			print_debug();

	inline uint16_t		system_status() const {return _status;}  //系统状态。内联函数

private:
	device::Device		*_interface;  //设备接口

	// XXX
	unsigned		_hardware;		///< Hardware revision   硬件修正版本
	unsigned		_max_actuators;		///< Maximum # of actuators supported by PX4IO  io控制板支持的最大执行机构的数量
	unsigned		_max_controls;		///< Maximum # of controls supported by PX4IO  io支持的控制最大数量
	unsigned		_max_rc_input;		///< Maximum receiver channels supported by PX4IO  io支持的最大接收通道数量
	unsigned		_max_relays;		///< Maximum relays supported by PX4IO   io口支持的最大继电器数量
	unsigned		_max_transfer;		///< Maximum number of I2C transfers supported by PX4IO  //io口支持的最大I2c传输数量

	unsigned 		_update_interval;	///< Subscription interval limiting send rate  订阅间隔限制发送速率
	bool			_rc_handling_disabled;	///< If set, IO does not evaluate, but only forward the RC values
	unsigned		_rc_chan_count;		///< Internal copy of the last seen number of RC channels
	uint64_t		_rc_last_valid;		///< last valid timestamp  最后一次有效的时间戳

	volatile int		_task;			///< worker task id  工作任务id号
	volatile bool		_task_should_exit;	///< worker terminate flag

	int			_mavlink_fd;		///< mavlink file descriptor.   mavlink文件描述

	perf_counter_t		_perf_update;		///< local performance counter for status updates
	perf_counter_t		_perf_write;		///< local performance counter for PWM control writes
	perf_counter_t		_perf_sample_latency;	///< total system latency (based on passed-through timestamp)

	/* cached IO state */
	uint16_t		_status;		///< Various IO status flags  io状态标志
	uint16_t		_alarms;		///< Various IO alarms  io警告状态
	uint16_t		_last_written_arming_s;	///< the last written arming state reg
	uint16_t		_last_written_arming_c;	///< the last written arming state reg

    /* subscribed topics */  //订阅主题
    int			_t_actuator_controls_0;	///< actuator controls group 0 topic  //4个控制组
	int			_t_actuator_controls_1;	///< actuator controls group 1 topic
	int			_t_actuator_controls_2;	///< actuator controls group 2 topic
	int			_t_actuator_controls_3;	///< actuator controls group 3 topic
    int			_t_actuator_armed;	///< system armed control topic  系统解锁控制主题
    int 			_t_vehicle_control_mode;///< vehicle control mode topic   飞行器控制模式主题
    int			_t_param;		///< parameter update topic    参数更新主题
    bool			_param_update_force;	///< force a parameter update  强制参数更新
    int			_t_vehicle_command;	///< vehicle command topic    飞行器命令主题

    /* advertised topics */  //发布主题，从io板来的数据
    orb_advert_t 		_to_input_rc;		///< rc inputs from io   来自io的rc输入
	orb_advert_t		_to_outputs;		///< mixed servo outputs topic
    orb_advert_t		_to_battery;		///< battery status / voltage   //电池状态和电压
	orb_advert_t		_to_servorail;		///< servorail status
    orb_advert_t		_to_safety;		///< status of safety   安全状态
    orb_advert_t 		_to_mixer_status; 	///< mixer status flags   混控状态标志

    actuator_outputs_s	_outputs;		///< mixed outputs   混控输出，执行机构输出
	servorail_status_s	_servorail_status;	///< servorail status

	bool			_primary_pwm_device;	///< true if we are the default PWM output  如果为帧，设置默认的pwm输出
	bool			_lockdown_override;	///< allow to override the safety lockdown  允许覆盖安全锁定

    float			_battery_amp_per_volt;	///< current sensor amps/volt  电流传感器
    float			_battery_amp_bias;	///< current sensor bias   电流传感器偏差
	float			_battery_mamphour_total;///< amp hours consumed so far
	uint64_t		_battery_last_timestamp;///< last amp hour calculation timestamp  最后一次电流校正时间戳
    bool			_cb_flighttermination;	///< true if the flight termination circuit breaker is enabled  如果飞行终止断路器使能
    bool 			_in_esc_calibration_mode;	///< do not send control outputs to IO (used for esc calibration)用于电调校准，不发送任何控制输出

    int32_t			_rssi_pwm_chan; ///< RSSI PWM input channel   输入通道接收器信号强度
    int32_t			_rssi_pwm_max; ///< max RSSI input on PWM channel   最大信号强度
    int32_t			_rssi_pwm_min; ///< min RSSI input on PWM channel   最小信号强度

	/**
     * Trampoline to the worker task  工作任务跳板
	 */
	static void		task_main_trampoline(int argc, char *argv[]);

	/**
	 * worker task
	 */
	void			task_main();  //主任务

	/**
     * Send controls for one group to IO  发送一个控制组到io
	 */
	int			io_set_control_state(unsigned group);

	/**
     * Send all controls to IO   发送所有的控制组到io
	 */
	int			io_set_control_groups();

	/**
     * Update IO's arming-related state  更新io解锁相关的状态
	 */
	int			io_set_arming_state();

	/**
     * Push RC channel configuration to IO.   发布rc通道配置到io口
	 */
	int			io_set_rc_config();

	/**
     * Fetch status and alarms from IO    从io口获取状态和解锁
	 *
     * Also publishes battery voltage/current. 同事发布电池电压和电流
	 */
	int			io_get_status();

	/**
     * Disable RC input handling   禁止rc输入处理
	 */
	int			io_disable_rc_handling();

	/**
     * Fetch RC inputs from IO.   从io口获得rc输入
	 *
	 * @param input_rc	Input structure to populate.
	 * @return		OK if data was returned.
	 */
	int			io_get_raw_rc_input(rc_input_values &input_rc);

	/**
	 * Fetch and publish raw RC input data.  获取并且发布原始rc输入数据
	 */
	int			io_publish_raw_rc();

	/**
	 * Fetch and publish the PWM servo outputs.  获取并且发布pwm伺服输出
	 */
	int			io_publish_pwm_outputs();

	/**
	 * write register(s)  写寄存器
	 *
	 * @param page		Register page to write to.
	 * @param offset	Register offset to start writing at.
	 * @param values	Pointer to array of values to write.
	 * @param num_values	The number of values to write.
	 * @return		OK if all values were successfully written.
	 */
	int			io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);

	/**
	 * write a register  函数重载
	 *
	 * @param page		Register page to write to.
	 * @param offset	Register offset to write to.
	 * @param value		Value to write.
	 * @return		OK if the value was written successfully.
	 */
	int			io_reg_set(uint8_t page, uint8_t offset, const uint16_t value);

	/**
	 * read register(s)
	 *
	 * @param page		Register page to read from.
	 * @param offset	Register offset to start reading from.
	 * @param values	Pointer to array where values should be stored.
	 * @param num_values	The number of values to read.
	 * @return		OK if all values were successfully read.
	 */
	int			io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values);

	/**
     * read a register  //读取伪寄存器
	 *
	 * @param page		Register page to read from.
	 * @param offset	Register offset to start reading from.
	 * @return		Register value that was read, or _io_reg_get_error on error.
	 */
	uint32_t		io_reg_get(uint8_t page, uint8_t offset);
	static const uint32_t	_io_reg_get_error = 0x80000000;

	/**
	 * modify a register
	 *
	 * @param page		Register page to modify.
	 * @param offset	Register offset to modify.
	 * @param clearbits	Bits to clear in the register.
	 * @param setbits	Bits to set in the register.
	 */
	int			io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits);

	/**
	 * Send mixer definition text to IO  发送混控器定义文本到io
	 */
	int			mixer_send(const char *buf, unsigned buflen, unsigned retries = 3);

	/**
	 * Handle a status update from IO.  处理一个来自io口的状态更新
	 *
	 * Publish IO status information if necessary.  如果必要，发布io状态信息
	 *
	 * @param status	The status register
	 */
	int			io_handle_status(uint16_t status);

	/**
     * Handle an alarm update from IO.   从io口处理一个解锁更新
	 *
     * Publish IO alarm information if necessary.  如果有必要，发布io解锁信息
	 *
	 * @param alarm		The status register
	 */
	int			io_handle_alarms(uint16_t alarms);

	/**
	 * Handle issuing dsm bind ioctl to px4io.
	 *
	 * @param dsmMode	0:dsm2, 1:dsmx
	 */
	void			dsm_bind_ioctl(int dsmMode);

	/**
     * Handle a battery update from IO.  从io口处理一个电池更新
	 *
	 * Publish IO battery information if necessary.
	 *
	 * @param vbatt		vbatt register
	 * @param ibatt		ibatt register
	 */
	void			io_handle_battery(uint16_t vbatt, uint16_t ibatt);

	/**
	 * Handle a servorail update from IO.
	 *
	 * Publish servo rail information if necessary.
	 *
	 * @param vservo	vservo register
	 * @param vrssi 	vrssi register
	 */
	void			io_handle_vservo(uint16_t vservo, uint16_t vrssi);

	/* do not allow to copy this class due to ptr data members */
	PX4IO(const PX4IO &);
	PX4IO operator=(const PX4IO &);
};

namespace  //匿名名字空间
{

PX4IO	*g_dev = nullptr;

}

PX4IO::PX4IO(device::Device *interface) :
	CDev("px4io", PX4IO_DEVICE_PATH),  //字符设备初始化
	_interface(interface),  //设备接口，应该是串口
	_hardware(0),
	_max_actuators(0),
	_max_controls(0),
	_max_rc_input(0),
	_max_relays(0),
	_max_transfer(16),	/* sensible default */
	_update_interval(0),
	_rc_handling_disabled(false),
	_rc_chan_count(0),
	_rc_last_valid(0),
	_task(-1),
	_task_should_exit(false),
	_mavlink_fd(-1),
	_perf_update(perf_alloc(PC_ELAPSED, "io update")),  //io更新时间花费
	_perf_write(perf_alloc(PC_ELAPSED, "io write")),  //io写更新时间花费
	_perf_sample_latency(perf_alloc(PC_ELAPSED, "io latency")),  //io延时时间花费
	_status(0),
	_alarms(0),
	_last_written_arming_s(0),
	_last_written_arming_c(0),
	_t_actuator_controls_0(-1),
	_t_actuator_controls_1(-1),
	_t_actuator_controls_2(-1),
	_t_actuator_controls_3(-1),
	_t_actuator_armed(-1),
	_t_vehicle_control_mode(-1),
	_t_param(-1),
	_param_update_force(false),
	_t_vehicle_command(-1),
	_to_input_rc(nullptr),
	_to_outputs(nullptr),
	_to_battery(nullptr),
	_to_servorail(nullptr),
	_to_safety(nullptr),
	_to_mixer_status(nullptr),
	_outputs{},
	_servorail_status{},
	_primary_pwm_device(false),
	_lockdown_override(false),
	_battery_amp_per_volt(90.0f / 5.0f), // this matches the 3DR current sensor
	_battery_amp_bias(0),
	_battery_mamphour_total(0),
	_battery_last_timestamp(0),
	_cb_flighttermination(true),
	_in_esc_calibration_mode(false),
	_rssi_pwm_chan(0),
	_rssi_pwm_max(0),
	_rssi_pwm_min(0)

{
	/* we need this potentially before it could be set in task_main */
	g_dev = this;

	_debug_enabled = false;
	_servorail_status.rssi_v = 0;
}

PX4IO::~PX4IO()
{
	/* tell the task we want it to go away */
	_task_should_exit = true; //告诉任务，要退出

	/* spin waiting for the task to stop */  //等待任务停止
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1) {
		task_delete(_task);  //删除任务
	}

	if (_interface != nullptr) {
		delete _interface;
	}

	/* deallocate perfs */
	perf_free(_perf_update);
	perf_free(_perf_write);
	perf_free(_perf_sample_latency);

	g_dev = nullptr;
}

int
PX4IO::detect()   //检测io模块是否挂上
{
	int ret;

	if (_task == -1) {  //任务没有启动

		/* do regular cdev init */
        ret = CDev::init();  //字符设备初始化

		if (ret != OK) { //字符设备初始化是否ok
			return ret;
		}

		/* get some parameters */  //获取io口协议版本号
		unsigned protocol = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION);

        if (protocol != PX4IO_PROTOCOL_VERSION) {  //如果协议版本不对
			if (protocol == _io_reg_get_error) {
				DEVICE_LOG("IO not installed");

			} else {
				DEVICE_LOG("IO version error");
				mavlink_log_emergency(_mavlink_fd, "IO VERSION MISMATCH, PLEASE UPGRADE SOFTWARE!");
			}

			return -1;
		}
	}

	DEVICE_LOG("IO found");  //找到io板

	return 0;
}

int
PX4IO::init(bool rc_handling_disabled)
{
    _rc_handling_disabled = rc_handling_disabled;  //设置rc是否处理
	return init();
}

int
PX4IO::init()  //io初始化
{
	int ret;
	param_t sys_restart_param;  //系统复位参数
	int32_t sys_restart_val = DM_INIT_REASON_VOLATILE;  //系统复位变量

	ASSERT(_task == -1);

	sys_restart_param = param_find("SYS_RESTART_TYPE");  //查找参数，系统复位类型

	if (sys_restart_param != PARAM_INVALID) {
		/* Indicate restart type is unknown */  //指示复位类型未知
		int32_t prev_val;
		param_get(sys_restart_param, &prev_val);

		if (prev_val != DM_INIT_REASON_POWER_ON) {
			param_set_no_notification(sys_restart_param, &sys_restart_val);
		}
	}

	/* do regular cdev init */
	ret = CDev::init();  //字符设备初始化

	if (ret != OK) {
		return ret;
	}

	/* get some parameters */
	unsigned protocol;
	hrt_abstime start_try_time = hrt_absolute_time();  //开始尝试时间

	do {
		usleep(2000);
		protocol = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION);  //协议版本
	} while (protocol == _io_reg_get_error && (hrt_elapsed_time(&start_try_time) < 700U * 1000U));

	/* if the error still persists after timing out, we give up */  //如果读取协议版本号错误超时，放弃
	if (protocol == _io_reg_get_error) {
		mavlink_and_console_log_emergency(_mavlink_fd, "Failed to communicate with IO, abort.");
		return -1;
	}

	if (protocol != PX4IO_PROTOCOL_VERSION) {  //如果控制协议版本不对
		mavlink_and_console_log_emergency(_mavlink_fd, "IO protocol/firmware mismatch, abort.");
		return -1;
	}

	_hardware      = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_HARDWARE_VERSION);  //从io板中获取信息
	_max_actuators = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_ACTUATOR_COUNT);
	_max_controls  = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_CONTROL_COUNT);
	_max_relays    = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RELAY_COUNT);
	_max_transfer  = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_MAX_TRANSFER) - 2;
	_max_rc_input  = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RC_INPUT_COUNT);

	if ((_max_actuators < 1) || (_max_actuators > 255) ||
	    (_max_relays > 32)   ||
	    (_max_transfer < 16) || (_max_transfer > 255)  ||
	    (_max_rc_input < 1)  || (_max_rc_input > 255)) {

		DEVICE_LOG("config read error");
		mavlink_log_emergency(_mavlink_fd, "[IO] config read fail, abort.");
		return -1;
	}

	if (_max_rc_input > input_rc_s::RC_INPUT_MAX_CHANNELS) {  //限制最大rc输入数量
		_max_rc_input = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	param_get(param_find("RC_RSSI_PWM_CHAN"), &_rssi_pwm_chan);
	param_get(param_find("RC_RSSI_PWM_MAX"), &_rssi_pwm_max);
	param_get(param_find("RC_RSSI_PWM_MIN"), &_rssi_pwm_min);

	/*
	 * Check for IO flight state - if FMU was flagged to be in  检查io飞行状态，如果fmu被标记在解锁状态
	 * armed state, FMU is recovering from an in-air reset.   fmu正在回复 来自一个在空中复位
	 * Read back status and request the commander to arm
	 * in this case.
	 */

	uint16_t reg;

	/* get IO's last seen FMU state */
	ret = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, &reg, sizeof(reg));

	if (ret != OK) {
		return ret;
	}

	/*
	 * in-air restart is only tried if the IO board reports it is  如果io控制板发布已经解锁并且配置了在空中复位，空中复位才被尝试
	 * already armed, and has been configured for in-air restart
	 */
	if ((reg & PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK) &&
	    (reg & PX4IO_P_SETUP_ARMING_FMU_ARMED)) {

		/* get a status update from IO */
		io_get_status();  //获取来自io口的状态更新

		mavlink_and_console_log_emergency(_mavlink_fd, "RECOVERING FROM FMU IN-AIR RESTART");  //mavlink日志

		/* WARNING: COMMANDER app/vehicle status must be initialized.
		 * If this fails (or the app is not started), worst-case IO
		 * remains untouched (so manual override is still available).
		 */

		int safety_sub = orb_subscribe(ORB_ID(actuator_armed));  //订阅飞行器解锁主题
		/* fill with initial values, clear updated flag */
		struct actuator_armed_s safety;
		uint64_t try_start_time = hrt_absolute_time();
		bool updated = false;

		/* keep checking for an update, ensure we got a arming information,
		   not something that was published a long time ago. */
		do {
			orb_check(safety_sub, &updated);  //检查解锁是否更新

			if (updated) { //只有解锁状态更新了才退出这个循环
				/* got data, copy and exit loop */
				orb_copy(ORB_ID(actuator_armed), safety_sub, &safety);  //更新数据
				break;
			}

			/* wait 10 ms */
			usleep(10000);

			/* abort after 5s */
			if ((hrt_absolute_time() - try_start_time) / 1000 > 3000) { //不然5s后终止
				mavlink_and_console_log_emergency(_mavlink_fd, "Failed to recover from in-air restart (1), abort");
				return 1;
			}

		} while (true);

		/* send command to arm system via command API */
		vehicle_command_s cmd;
		/* send this to itself */
		param_t sys_id_param = param_find("MAV_SYS_ID");  //通过命令api，发送命令到解锁系统
		param_t comp_id_param = param_find("MAV_COMP_ID");

		int32_t sys_id;
		int32_t comp_id;

		if (param_get(sys_id_param, &sys_id)) {  //获取参数
			errx(1, "PRM SYSID");
		}

		if (param_get(comp_id_param, &comp_id)) {
			errx(1, "PRM CMPID");
		}

		cmd.target_system = sys_id;  //目标系统id
		cmd.target_component = comp_id;  //目标不见id
		cmd.source_system = sys_id;   //源系统id
		cmd.source_component = comp_id;  //源目标id
		/* request arming */  //请求解锁
		cmd.param1 = 1.0f;
		cmd.param2 = 0;
		cmd.param3 = 0;
		cmd.param4 = 0;
		cmd.param5 = 0;
		cmd.param6 = 0;
		cmd.param7 = 0;
		cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;

		/* ask to confirm command */  //应答确认命令
		cmd.confirmation =  1;

		/* send command once */
		orb_advert_t pub = orb_advertise(ORB_ID(vehicle_command), &cmd);  //发送一次命令

		/* spin here until IO's state has propagated into the system */
		do {  //在此循环，直到io状态传到系统
			orb_check(safety_sub, &updated);  //检查安全开关

			if (updated) {
				orb_copy(ORB_ID(actuator_armed), safety_sub, &safety);  //复制安全开关状态
			}

			/* wait 50 ms */
			usleep(50000);

			/* abort after 5s */
			if ((hrt_absolute_time() - try_start_time) / 1000 > 2000) {  //5s后终止
				mavlink_and_console_log_emergency(_mavlink_fd, "Failed to recover from in-air restart (2), abort");
				return 1;
			}

			/* re-send if necessary */
			if (!safety.armed) {  //如果需要，重新发布飞行器状态
				orb_publish(ORB_ID(vehicle_command), pub, &cmd);
				DEVICE_LOG("re-sending arm cmd");
			}

			/* keep waiting for state change for 2 s */
		} while (!safety.armed);

		/* Indicate restart type is in-flight */  //指示在空中复位类型
		sys_restart_val = DM_INIT_REASON_IN_FLIGHT;
		int32_t prev_val;
		param_get(sys_restart_param, &prev_val);  //获取参数

		if (prev_val != sys_restart_val) {
			param_set(sys_restart_param, &sys_restart_val);
		}

		/* regular boot, no in-air restart, init IO */

	} else {

		/* dis-arm IO before touching anything */  //在接触任何事情之前对io板加锁
		io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING,
			      PX4IO_P_SETUP_ARMING_FMU_ARMED |
			      PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK |
			      PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK |
			      PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE |
			      PX4IO_P_SETUP_ARMING_LOCKDOWN, 0);  //修改寄存器

		if (_rc_handling_disabled) {  //吐过rc处理禁止
			ret = io_disable_rc_handling();

			if (ret != OK) {
				DEVICE_LOG("failed disabling RC handling");
				return ret;
			}

		} else {
			/* publish RC config to IO */
			ret = io_set_rc_config();  //发布rc配置到口

			if (ret != OK) {
				mavlink_and_console_log_critical(_mavlink_fd, "IO RC config upload fail");
				return ret;
			}
		}

		/* Indicate restart type is power on */
		sys_restart_val = DM_INIT_REASON_POWER_ON;  //指定复位类型为上点复位
		int32_t prev_val;
		param_get(sys_restart_param, &prev_val);  //获取系统复位参数

		if (prev_val != sys_restart_val) {
			param_set(sys_restart_param, &sys_restart_val);  //参数设置，系统复位参数
		}

	}

	/* set safety to off if circuit breaker enabled */  //如果断路器使能，设置安全开关关闭
	if (circuit_breaker_enabled("CBRK_IO_SAFETY", CBRK_IO_SAFETY_KEY)) {
		(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_OFF, PX4IO_FORCE_SAFETY_MAGIC);
	}

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */  //尝试声明一般的pwm输出设备节点
	ret = register_driver(PWM_OUTPUT0_DEVICE_PATH, &fops, 0666, (void *)this);  //注册一个设备驱动到伪文件系统

	if (ret == OK) {  //注册成功
		DEVICE_LOG("default PWM output device");
		_primary_pwm_device = true;
	}

	/* start the IO interface task */  //启动io接口任务
	_task = px4_task_spawn_cmd("px4io",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_ACTUATOR_OUTPUTS,
				   1500,
				   (main_t)&PX4IO::task_main_trampoline,  //回调函数
				   nullptr);

	if (_task < 0) {  //任务创建失败
		DEVICE_DEBUG("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}

void
PX4IO::task_main_trampoline(int argc, char *argv[])  //主任务跳板
{
	g_dev->task_main();
}

void
PX4IO::task_main()  //主任务
{
	hrt_abstime poll_last = 0;
	hrt_abstime orb_check_last = 0;

	_mavlink_fd = ::open(MAVLINK_LOG_DEVICE, 0);  //打开mavlink日志设备

	/*
	 * Subscribe to the appropriate PWM output topic based on whether we are the
	 * primary PWM output or not.  订阅合适的pwm输出主题基于是否是基本pwm输出
	 */
	_t_actuator_controls_0 = orb_subscribe(ORB_ID(actuator_controls_0));  //控制组0主题订阅
	orb_set_interval(_t_actuator_controls_0, 20);		/* default to 50Hz */
	_t_actuator_controls_1 = orb_subscribe(ORB_ID(actuator_controls_1));
	orb_set_interval(_t_actuator_controls_1, 33);		/* default to 30Hz */
	_t_actuator_controls_2 = orb_subscribe(ORB_ID(actuator_controls_2));
	orb_set_interval(_t_actuator_controls_2, 33);		/* default to 30Hz */
	_t_actuator_controls_3 = orb_subscribe(ORB_ID(actuator_controls_3));
	orb_set_interval(_t_actuator_controls_3, 33);		/* default to 30Hz */
	_t_actuator_armed = orb_subscribe(ORB_ID(actuator_armed));  //执行机构解锁主题
	_t_vehicle_control_mode = orb_subscribe(ORB_ID(vehicle_control_mode));  //飞行器控制模式主题
	_t_param = orb_subscribe(ORB_ID(parameter_update));   //参数更新主题
	_t_vehicle_command = orb_subscribe(ORB_ID(vehicle_command));  //飞行器命令主题

	if ((_t_actuator_controls_0 < 0) ||
	    (_t_actuator_armed < 0) ||
	    (_t_vehicle_control_mode < 0) ||
	    (_t_param < 0) ||
	    (_t_vehicle_command < 0)) {
		warnx("subscription(s) failed");
		goto out;
	}  //任何一个主题失败都要退出

	/* Fetch initial flight termination circuit breaker state */  //获取初始飞行终止断路器状态
	_cb_flighttermination = circuit_breaker_enabled("CBRK_FLIGHTTERM", CBRK_FLIGHTTERM_KEY);  //飞行终止按钮

	/* poll descriptor */  //轮询描述
	pollfd fds[1];
	fds[0].fd = _t_actuator_controls_0;  //执行机构控制组0
	fds[0].events = POLLIN;

	_param_update_force = true;  //参数强行更新

	/* lock against the ioctl handler */
	lock();

	/* loop talking to IO */
	while (!_task_should_exit) {  //任务循环退出标志

		/* adjust update interval */  //自适应更新间隔
		if (_update_interval != 0) {
			if (_update_interval < UPDATE_INTERVAL_MIN) { //限制最小的更新间隔
				_update_interval = UPDATE_INTERVAL_MIN;
			}

			if (_update_interval > 100) { //限制最大的更新间隔
				_update_interval = 100;
			}

			orb_set_interval(_t_actuator_controls_0, _update_interval);  //设置控制组0更新间隔
			/*
			 * NOT changing the rate of groups 1-3 here, because only attitude
			 * really needs to run fast.
			 */
			_update_interval = 0;
		}

		/* sleep waiting for topic updates, but no more than 20ms */
		unlock();
		int ret = ::poll(fds, 1, 20);  //对控制组0进行轮询
		lock();

		/* this would be bad... */
		if (ret < 0) {  //如果轮询设置失败
			warnx("poll error %d", errno);
			continue;
		}

		perf_begin(_perf_update);
		hrt_abstime now = hrt_absolute_time();  //获取当前绝对时间

		/* if we have new control data from the ORB, handle it */  //ORB如果有新的控制数据就进行处理
		if (fds[0].revents & POLLIN) { //如果为轮询

			/* we're not nice to the lower-priority control groups and only check them
			   when the primary group updated (which is now). */
			(void)io_set_control_groups();
		}

		if (now >= poll_last + IO_POLL_INTERVAL) {  //如果现在的时间大于上次时间加轮询间隔
			/* run at 50-250Hz */
			poll_last = now;

			/* pull status and alarms from IO */
			io_get_status(); //获取io状态

			/* get raw R/C input from IO */
			io_publish_raw_rc();  //获取来自io原始rc输入

			/* fetch PWM outputs from IO */
			io_publish_pwm_outputs();  //从io口获取pwm输出

			/* check updates on uORB topics and handle it */
			bool updated = false;

			/* arming state */
			orb_check(_t_actuator_armed, &updated); //检查解锁状态

			if (!updated) {  //如果解锁没有更新
				orb_check(_t_vehicle_control_mode, &updated);  //检查飞行器控制模式是否跟新
			}

			if (updated) {
				io_set_arming_state(); //设置io解锁状态
			}
		}

		if (now >= orb_check_last + ORB_CHECK_INTERVAL) {
			/* run at 5Hz */
			orb_check_last = now;

			/* try to claim the MAVLink log FD */
			if (_mavlink_fd < 0) {
				_mavlink_fd = ::open(MAVLINK_LOG_DEVICE, 0);
			}

			/* check updates on uORB topics and handle it */
			bool updated = false;

			/* vehicle command */
			orb_check(_t_vehicle_command, &updated);  //检查飞行器命令是否更新

			if (updated) { //如果更新
				struct vehicle_command_s cmd;
				orb_copy(ORB_ID(vehicle_command), _t_vehicle_command, &cmd);  //复制飞行器命令

				// Check for a DSM pairing command  检查dsm解析命令
				if (((unsigned int)cmd.command == vehicle_command_s::VEHICLE_CMD_START_RX_PAIR) && ((int)cmd.param1 == 0)) {
					dsm_bind_ioctl((int)cmd.param2);
				}
			}

			/*
			 * If parameters have changed, re-send RC mappings to IO  如果参数已经改变，重新发送rc映射到io
			 *
			 * XXX this may be a bit spammy
			 */
			orb_check(_t_param, &updated);  //检查参数更新主题

			if (updated || _param_update_force) {  //如果参数更新或者强行参数更新
				_param_update_force = false;  //清除强制更新
				parameter_update_s pupdate;
				orb_copy(ORB_ID(parameter_update), _t_param, &pupdate);  //复制参数

				int32_t dsm_bind_val;
				param_t dsm_bind_param;

				/* see if bind parameter has been set, and reset it to -1 */
				param_get(dsm_bind_param = param_find("RC_DSM_BIND"), &dsm_bind_val);

				if (dsm_bind_val > -1) {
					dsm_bind_ioctl(dsm_bind_val);
					dsm_bind_val = -1;
					param_set(dsm_bind_param, &dsm_bind_val);
				}

				/* re-upload RC input config as it may have changed */
				io_set_rc_config(); //重新上传rc输入配置，如果它可能已经改变

				/* re-set the battery scaling */  //重置电池缩放因子
				int32_t voltage_scaling_val;
				param_t voltage_scaling_param;

				/* set battery voltage scaling */  //获取电池电压缩放因子
				param_get(voltage_scaling_param = param_find("BAT_V_SCALE_IO"), &voltage_scaling_val);

				/* send scaling voltage to IO */  //发送电压缩放到io
				uint16_t scaling = voltage_scaling_val;
				int pret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_VBATT_SCALE, &scaling, 1);

				if (pret != OK) {  //判断是否设置成功
					mavlink_and_console_log_critical(_mavlink_fd, "IO vscale upload failed");
				}

				/* send RC throttle failsafe value to IO */  //设置rc失效保护阈值到io
				int32_t failsafe_param_val;
				param_t failsafe_param = param_find("RC_FAILS_THR");  //获取

				if (failsafe_param != PARAM_INVALID) {

					param_get(failsafe_param, &failsafe_param_val);//获取失效保护参数

					if (failsafe_param_val > 0) {

						uint16_t failsafe_thr = failsafe_param_val;
						pret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_RC_THR_FAILSAFE_US, &failsafe_thr, 1);  //设置rc失效保护阈值

						if (pret != OK) {
							mavlink_and_console_log_critical(_mavlink_fd, "failsafe upload failed, FS: %d us", (int)failsafe_thr);
						}
					}
				}

				int32_t safety_param_val;
				param_t safety_param = param_find("RC_FAILS_THR");

				if (safety_param != PARAM_INVALID) {  //如果安全参数有效

					param_get(safety_param, &safety_param_val);

					if (safety_param_val == PX4IO_FORCE_SAFETY_MAGIC) {
						/* disable IO safety if circuit breaker asked for it */
						(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_OFF, safety_param_val);  //获取
					}
				}

				/* Check if the flight termination circuit breaker has been updated */  //检查飞行终止断路器是否已经更新
				_cb_flighttermination = circuit_breaker_enabled("CBRK_FLIGHTTERM", CBRK_FLIGHTTERM_KEY);

				param_get(param_find("RC_RSSI_PWM_CHAN"), &_rssi_pwm_chan);
				param_get(param_find("RC_RSSI_PWM_MAX"), &_rssi_pwm_max);
				param_get(param_find("RC_RSSI_PWM_MIN"), &_rssi_pwm_min);

				/*
				 * Set invert mask for PWM outputs (does not apply to S.Bus)
				 */
				int16_t pwm_invert_mask = 0;

				for (unsigned i = 0; i < _max_actuators; i++) {  //都每个执行机构进行扫描
					char pname[16];
					int32_t ival;

					/* fill the channel reverse mask from parameters */
					sprintf(pname, "PWM_MAIN_REV%d", i + 1);
					param_t param_h = param_find(pname);

					if (param_h != PARAM_INVALID) {  //参数有效
						param_get(param_h, &ival);  //取出参数
						pwm_invert_mask |= ((int16_t)(ival != 0)) << i;  //反向掩码
					}
				}

				(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_REVERSE, pwm_invert_mask);

				float trim_val;
				param_t parm_handle;

				parm_handle = param_find("TRIM_ROLL");  //roll裁剪

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &trim_val);
					(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_TRIM_ROLL, FLOAT_TO_REG(trim_val));
				}

				parm_handle = param_find("TRIM_PITCH");

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &trim_val);
					(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_TRIM_PITCH, FLOAT_TO_REG(trim_val));
				}

				parm_handle = param_find("TRIM_YAW");

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &trim_val);
					(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_TRIM_YAW, FLOAT_TO_REG(trim_val));
				}

				/* S.BUS output */
				int sbus_mode;
				parm_handle = param_find("PWM_SBUS_MODE");

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &sbus_mode);

					if (sbus_mode == 1) {
						/* enable S.BUS 1 */
						(void)io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_SBUS1_OUT);

					} else if (sbus_mode == 2) {
						/* enable S.BUS 2 */
						(void)io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_SBUS2_OUT);

					} else {
						/* disable S.BUS */
						(void)io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES,
								    (PX4IO_P_SETUP_FEATURES_SBUS1_OUT | PX4IO_P_SETUP_FEATURES_SBUS2_OUT), 0);
					}
				}
			}

		}

		perf_end(_perf_update);
	}

	unlock();

out:
	DEVICE_DEBUG("exiting");

	/* clean up the alternate device node */
	if (_primary_pwm_device) {
		unregister_driver(PWM_OUTPUT0_DEVICE_PATH);  //注销pwm输出设备0
	}

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

int
PX4IO::io_set_control_groups()  //设置控制组
{
	int ret = io_set_control_state(0);  //向io设置控制组状态

	/* send auxiliary control groups */  //发送辅助控制组
	(void)io_set_control_state(1);
	(void)io_set_control_state(2);
	(void)io_set_control_state(3);

	return ret;
}

int
PX4IO::io_set_control_state(unsigned group)  //设置io控制状态
{
	actuator_controls_s	controls;	///< actuator outputs  执行机构输出
	uint16_t 		regs[_max_actuators];

	/* get controls */  //获取控制
	bool changed = false;

	switch (group) {  //旋转控制组
	case 0: {
			orb_check(_t_actuator_controls_0, &changed); //检查控制组0是否更新

			if (changed) {
				orb_copy(ORB_ID(actuator_controls_0), _t_actuator_controls_0, &controls);
				perf_set(_perf_sample_latency, hrt_elapsed_time(&controls.timestamp_sample));
			}
		}
		break;

	case 1: {
			orb_check(_t_actuator_controls_1, &changed);

			if (changed) {
				orb_copy(ORB_ID(actuator_controls_1), _t_actuator_controls_1, &controls);
			}
		}
		break;

	case 2: {
			orb_check(_t_actuator_controls_2, &changed);

			if (changed) {
				orb_copy(ORB_ID(actuator_controls_2), _t_actuator_controls_2, &controls);
			}
		}
		break;

	case 3: {
			orb_check(_t_actuator_controls_3, &changed);

			if (changed) {
				orb_copy(ORB_ID(actuator_controls_3), _t_actuator_controls_3, &controls);
			}
		}
		break;
	}

	if (!changed && (!_in_esc_calibration_mode || group != 0)) {  //如果没有控制组更新并且不在电调校准模式或者控制组不为空
		return -1;

	} else if (_in_esc_calibration_mode && group == 0) {  //如果处于电调校准模式并且控制组为空
		/* modify controls to get max pwm (full thrust) on every esc */  //修改控制取获得最大pwm为每个电调
		memset(&controls, 0, sizeof(controls));

		/* set maximum thrust */
		controls.control[3] = 1.0f;  //设置最大的推力
	}

	for (unsigned i = 0; i < _max_controls; i++) {  //对控制器进行轮询

		/* ensure FLOAT_TO_REG does not produce an integer overflow */  //保证浮点到整型转换不会产生整型溢出
		float ctrl = controls.control[i];

		if (ctrl < -1.0f) {  //限制控制范围
			ctrl = -1.0f;

		} else if (ctrl > 1.0f) {
			ctrl = 1.0f;
		}

		regs[i] = FLOAT_TO_REG(ctrl);  //转换为整型
	}

	/* copy values to registers in IO */
	return io_reg_set(PX4IO_PAGE_CONTROLS, group * PX4IO_PROTOCOL_MAX_CONTROL_COUNT, regs, _max_controls);  //传到io控制板
}


int
PX4IO::io_set_arming_state()  //设置io口解锁状态
{
	actuator_armed_s	armed;		///< system armed state  系统解锁状态
	vehicle_control_mode_s	control_mode;	///< vehicle_control_mode  飞行器控制模式

	int have_armed = orb_copy(ORB_ID(actuator_armed), _t_actuator_armed, &armed);  //获取解锁信息
	int have_control_mode = orb_copy(ORB_ID(vehicle_control_mode), _t_vehicle_control_mode, &control_mode);  //获取控制模式
	_in_esc_calibration_mode = armed.in_esc_calibration_mode;  //处于电调校正模式

	uint16_t set = 0;
	uint16_t clear = 0;

	if (have_armed == OK) {  //如果已经解锁
		_in_esc_calibration_mode = armed.in_esc_calibration_mode;

		if (armed.armed || _in_esc_calibration_mode) { //如果解锁或者处于电调校准模式
			set |= PX4IO_P_SETUP_ARMING_FMU_ARMED;  //设置fmu解锁状态

		} else {
			clear |= PX4IO_P_SETUP_ARMING_FMU_ARMED;
		}

		if (armed.lockdown && !_lockdown_override) {   //解锁锁定并且锁定不能被覆盖
			set |= PX4IO_P_SETUP_ARMING_LOCKDOWN;  //设置解锁锁定
			_lockdown_override = true;

		} else if (!armed.lockdown && _lockdown_override) {
			clear |= PX4IO_P_SETUP_ARMING_LOCKDOWN;
			_lockdown_override = false;
		}

		/* Do not set failsafe if circuit breaker is enabled */  //如果断路器使能，不要设置失效保护
		if (armed.force_failsafe && !_cb_flighttermination) {  //强制失效保护并且
			set |= PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE;  //设置强制失效保护

		} else {
			clear |= PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE;
		}

		// XXX this is for future support in the commander 这个是在命令中为未来支持的
		// but can be removed if unneeded  但是如果不需要不能移除
		// if (armed.termination_failsafe) {
		// 	set |= PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE;
		// } else {
		// 	clear |= PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE;
		// }

		if (armed.ready_to_arm) {   //准备解锁
			set |= PX4IO_P_SETUP_ARMING_IO_ARM_OK;

		} else {
			clear |= PX4IO_P_SETUP_ARMING_IO_ARM_OK;
		}
	}

	if (have_control_mode == OK) {  //如果控制模式ok
		if (control_mode.flag_external_manual_override_ok) {
			set |= PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK;

		} else {
			clear |= PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK;
		}
	}

	if (_last_written_arming_s != set || _last_written_arming_c != clear) {
		_last_written_arming_s = set;
		_last_written_arming_c = clear;
		return io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, clear, set);
	}  //设置io解锁

	return 0;
}

int
PX4IO::disable_rc_handling()  //禁止rc处理
{
	_rc_handling_disabled = true;
	return io_disable_rc_handling();
}

int
PX4IO::io_disable_rc_handling()  //io禁止rc处理
{
	uint16_t set = PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED;
	uint16_t clear = 0;

	return io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, clear, set);  //修改寄存器
}

int
PX4IO::io_set_rc_config()  //设置rc配置
{
	unsigned offset = 0;
	int input_map[_max_rc_input];  //输入映射
	int32_t ichan;
	int ret = OK;

	/*
	 * Generate the input channel -> control channel mapping table; 生产输入通道控制通道映射表
	 * assign RC_MAP_ROLL/PITCH/YAW/THROTTLE to the canonical
	 * controls.
	 */

	/* fill the mapping with an error condition triggering value */  //填充映射，用错误状态触发值
	for (unsigned i = 0; i < _max_rc_input; i++) {
		input_map[i] = UINT8_MAX;
	}

	/*
	 * NOTE: The indices for mapped channels are 1-based
	 *       for compatibility reasons with existing
	 *       autopilots / GCS'. 通道映射指示器
	 */

	/* ROLL */
	param_get(param_find("RC_MAP_ROLL"), &ichan);  //参数获取

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 0;
	}

	/* PITCH */
	param_get(param_find("RC_MAP_PITCH"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 1;
	}

	/* YAW */
	param_get(param_find("RC_MAP_YAW"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 2;
	}

	/* THROTTLE */  //油门
	param_get(param_find("RC_MAP_THROTTLE"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 3;
	}

	/* FLAPS */
	param_get(param_find("RC_MAP_FLAPS"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 4;
	}

	/* AUX 1*/
	param_get(param_find("RC_MAP_AUX1"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 5;
	}

	/* AUX 2*/
	param_get(param_find("RC_MAP_AUX2"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 6;
	}

	/* AUX 3*/
	param_get(param_find("RC_MAP_AUX3"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 7;
	}

	/* MAIN MODE SWITCH */  //主要模式切换
	param_get(param_find("RC_MAP_MODE_SW"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		/* use out of normal bounds index to indicate special channel */
		input_map[ichan - 1] = PX4IO_P_RC_CONFIG_ASSIGNMENT_MODESWITCH;
	}

	/*
	 * Iterate all possible RC inputs.  迭代所有可能的rc输入
	 */
	for (unsigned i = 0; i < _max_rc_input; i++) {  //对所有输入轮询
		uint16_t regs[PX4IO_P_RC_CONFIG_STRIDE];
		char pname[16];
		float fval;

		/*
		 * RC params are floats, but do only  rc参数是浮点数，但是仅仅限制在整型值
		 * contain integer values. Do not scale  不要缩放或者扔掉他们，让自动类型转换干这个事情
		 * or cast them, let the auto-typeconversion
		 * do its job here.
		 * Channels: 500 - 2500
		 * Inverted flag: -1 (inverted) or 1 (normal)
		 */

		sprintf(pname, "RC%d_MIN", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_MIN] = fval;

		sprintf(pname, "RC%d_TRIM", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_CENTER] = fval;

		sprintf(pname, "RC%d_MAX", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_MAX] = fval;

		sprintf(pname, "RC%d_DZ", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_DEADZONE] = fval;

		regs[PX4IO_P_RC_CONFIG_ASSIGNMENT] = input_map[i];

		regs[PX4IO_P_RC_CONFIG_OPTIONS] = PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;
		sprintf(pname, "RC%d_REV", i + 1);
		param_get(param_find(pname), &fval);

		/*
		 * This has been taken for the sake of compatibility
		 * with APM's setup / mission planner: normal: 1,
		 * inverted: -1
		 */
		if (fval < 0) {
			regs[PX4IO_P_RC_CONFIG_OPTIONS] |= PX4IO_P_RC_CONFIG_OPTIONS_REVERSE;
		}

		/* send channel config to IO */  //发送通道配置到io
		ret = io_reg_set(PX4IO_PAGE_RC_CONFIG, offset, regs, PX4IO_P_RC_CONFIG_STRIDE);

		if (ret != OK) {
			DEVICE_LOG("rc config upload failed");
			break;
		}

		/* check the IO initialisation flag */
		if (!(io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS) & PX4IO_P_STATUS_FLAGS_INIT_OK)) {
			mavlink_and_console_log_critical(_mavlink_fd, "config for RC%d rejected by IO", i + 1);
			break;
		}

		offset += PX4IO_P_RC_CONFIG_STRIDE;
	}

	return ret;
}

int
PX4IO::io_handle_status(uint16_t status)  //io处理状态
{
	int ret = 1;
	/**
	 * WARNING: This section handles in-air resets.  这个部分处理空中复位
	 */

	/* check for IO reset - force it back to armed if necessary */  //检查io复位，强制它返回到解锁状态如果有需要
	if (_status & PX4IO_P_STATUS_FLAGS_SAFETY_OFF && !(status & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)
	    && !(status & PX4IO_P_STATUS_FLAGS_ARM_SYNC)) {
		/* set the arming flag */  //设置解锁标志
		ret = io_reg_modify(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, 0,
				    PX4IO_P_STATUS_FLAGS_SAFETY_OFF | PX4IO_P_STATUS_FLAGS_ARM_SYNC);

		/* set new status */
		_status = status;  //设置新的状态
		_status &= PX4IO_P_STATUS_FLAGS_SAFETY_OFF;

	} else if (!(_status & PX4IO_P_STATUS_FLAGS_ARM_SYNC)) {  //解锁同步标志

		/* set the sync flag */
		ret = io_reg_modify(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, 0, PX4IO_P_STATUS_FLAGS_ARM_SYNC);
		/* set new status */
		_status = status;

	} else {
		ret = 0;

		/* set new status */
		_status = status;
	}

	/**
	 * Get and handle the safety status  获取并且处理安全状态
	 */
	struct safety_s safety;
	safety.timestamp = hrt_absolute_time();  //时间戳

	if (status & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) {
		safety.safety_off = true;
		safety.safety_switch_available = true;

	} else {
		safety.safety_off = false;
		safety.safety_switch_available = true;
	}

	/* lazily publish the safety status */  //缓缓发布安全状态
	if (_to_safety != nullptr) {
		orb_publish(ORB_ID(safety), _to_safety, &safety);

	} else {
		_to_safety = orb_advertise(ORB_ID(safety), &safety);
	}

	return ret;
}

void
PX4IO::dsm_bind_ioctl(int dsmMode)  //dsm绑定io控制
{
	if (!(_status & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)) {  //安全关闭
		mavlink_log_info(_mavlink_fd, "[IO] binding DSM%s RX", (dsmMode == 0) ? "2" : ((dsmMode == 1) ? "-X" : "-X8"));
		int ret = ioctl(nullptr, DSM_BIND_START,
				(dsmMode == 0) ? DSM2_BIND_PULSES : ((dsmMode == 1) ? DSMX_BIND_PULSES : DSMX8_BIND_PULSES));

		if (ret) {
			mavlink_log_critical(_mavlink_fd, "binding failed.");
		}

	} else {
		mavlink_log_info(_mavlink_fd, "[IO] system armed, bind request rejected");
	}
}


int
PX4IO::io_handle_alarms(uint16_t alarms)  //io处理解锁
{

	/* XXX handle alarms */


	/* set new alarms state */
	_alarms = alarms;  //设置新的警告状态

	return 0;
}

void
PX4IO::io_handle_battery(uint16_t vbatt, uint16_t ibatt)  //io电池处理
{
	/* only publish if battery has a valid minimum voltage */  //如果电压有一个最小的有效电压值
	if (vbatt <= 4900) {
		return;
	}

	battery_status_s	battery_status;
	battery_status.timestamp = hrt_absolute_time();

	/* voltage is scaled to mV */
	battery_status.voltage_v = vbatt / 1000.0f;
	battery_status.voltage_filtered_v = vbatt / 1000.0f;

	/*
	  ibatt contains the raw ADC count, as 12 bit ADC
	  value, with full range being 3.3v
	*/
	battery_status.current_a = ibatt * (3.3f / 4096.0f) * _battery_amp_per_volt;
	battery_status.current_a += _battery_amp_bias;

	/*
	  integrate battery over time to get total mAh used
	*/
	if (_battery_last_timestamp != 0) {
		_battery_mamphour_total += battery_status.current_a *
					   (battery_status.timestamp - _battery_last_timestamp) * 1.0e-3f / 3600;
	}

	battery_status.discharged_mah = _battery_mamphour_total;
	_battery_last_timestamp = battery_status.timestamp;

	/* the announced battery status would conflict with the simulated battery status in HIL */
	if (!(_pub_blocked)) {
		/* lazily publish the battery voltage */
		if (_to_battery != nullptr) {
			orb_publish(ORB_ID(battery_status), _to_battery, &battery_status);

		} else {
			_to_battery = orb_advertise(ORB_ID(battery_status), &battery_status);
		}
	}
}

void
PX4IO::io_handle_vservo(uint16_t vservo, uint16_t vrssi)  //伺服处理
{
	_servorail_status.timestamp = hrt_absolute_time();  //时间戳

	/* voltage is scaled to mV */  //电压缩放到mv
	_servorail_status.voltage_v = vservo * 0.001f;
	_servorail_status.rssi_v    = vrssi * 0.001f;

	/* lazily publish the servorail voltages */
	if (_to_servorail != nullptr) {
		orb_publish(ORB_ID(servorail_status), _to_servorail, &_servorail_status);

	} else {
		_to_servorail = orb_advertise(ORB_ID(servorail_status), &_servorail_status);
	}
}

int
PX4IO::io_get_status()  //获取状态
{
	uint16_t	regs[6];
	int		ret;

	/* get
	 * STATUS_FLAGS, STATUS_ALARMS, STATUS_VBATT, STATUS_IBATT,
	 * STATUS_VSERVO, STATUS_VRSSI, STATUS_PRSSI
	 * in that order */
	ret = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, &regs[0], sizeof(regs) / sizeof(regs[0]));

	if (ret != OK) {
		return ret;
	}

	io_handle_status(regs[0]);  //处理状态
	io_handle_alarms(regs[1]);  //处理警告

	io_handle_vservo(regs[4], regs[5]);


	return ret;
}

int
PX4IO::io_get_raw_rc_input(rc_input_values &input_rc)  //io获取原始rc输入
{
	uint32_t channel_count;
	int	ret;

	/* we don't have the status bits, so input_source has to be set elsewhere */  //我们没有状态位，所以输入源不得不在任何地方设置
	input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_UNKNOWN;

	const unsigned prolog = (PX4IO_P_RAW_RC_BASE - PX4IO_P_RAW_RC_COUNT);
	uint16_t regs[input_rc_s::RC_INPUT_MAX_CHANNELS + prolog];

	/*
	 * Read the channel count and the first 9 channels.  读取通道数量并且是前9个通道
	 *
	 * This should be the common case (9 channel R/C control being a reasonable upper bound).
	 */
	ret = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT, &regs[0], prolog + 9);

	if (ret != OK) {
		return ret;
	}

	/*
	 * Get the channel count any any extra channels. This is no more expensive than reading the
	 * channel count once.获取通道数量，辅助通道
	 */
	channel_count = regs[PX4IO_P_RAW_RC_COUNT];

	/* limit the channel count */
	if (channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) { //限制通道数量
		channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	_rc_chan_count = channel_count;

	input_rc.timestamp_publication = hrt_absolute_time();  //rc时间戳

	input_rc.rc_ppm_frame_length = regs[PX4IO_P_RAW_RC_DATA];
	input_rc.rssi = regs[PX4IO_P_RAW_RC_NRSSI];
	input_rc.rc_failsafe = (regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
	input_rc.rc_lost = !(regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_RC_OK);
	input_rc.rc_lost_frame_count = regs[PX4IO_P_RAW_LOST_FRAME_COUNT];
	input_rc.rc_total_frame_count = regs[PX4IO_P_RAW_FRAME_COUNT];
	input_rc.channel_count = channel_count;

	/* rc_lost has to be set before the call to this function */
	if (!input_rc.rc_lost && !input_rc.rc_failsafe) {
		_rc_last_valid = input_rc.timestamp_publication;
	}

	input_rc.timestamp_last_signal = _rc_last_valid;

	/* FIELDS NOT SET HERE */
	/* input_rc.input_source is set after this call XXX we might want to mirror the flags in the RC struct */

	if (channel_count > 9) {
		ret = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE + 9, &regs[prolog + 9], channel_count - 9);

		if (ret != OK) {
			return ret;
		}
	}

	/* last thing set are the actual channel values as 16 bit values */
	for (unsigned i = 0; i < channel_count; i++) {
		input_rc.values[i] = regs[prolog + i];
	}

	/* get RSSI from input channel */
	if (_rssi_pwm_chan > 0 && _rssi_pwm_chan <= input_rc_s::RC_INPUT_MAX_CHANNELS && _rssi_pwm_max - _rssi_pwm_min != 0) {
		int rssi = ((input_rc.values[_rssi_pwm_chan - 1] - _rssi_pwm_min) * 100) /
			   (_rssi_pwm_max - _rssi_pwm_min);
		rssi = rssi > 100 ? 100 : rssi;
		rssi = rssi < 0 ? 0 : rssi;
		input_rc.rssi = rssi;
	}

	return ret;
}

int
PX4IO::io_publish_raw_rc()  //io发布原始rc
{

	/* fetch values from IO */  //从io口获取值
	rc_input_values	rc_val;

	/* set the RC status flag ORDER MATTERS! */
	rc_val.rc_lost = !(_status & PX4IO_P_STATUS_FLAGS_RC_OK);

	int ret = io_get_raw_rc_input(rc_val);

	if (ret != OK) {
		return ret;
	}

	/* sort out the source of the values */  //排列源头输出值
	if (_status & PX4IO_P_STATUS_FLAGS_RC_PPM) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM;

	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_DSM) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SPEKTRUM;

	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_SBUS) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SBUS;

	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_ST24) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_ST24;

	} else {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_UNKNOWN;

		/* only keep publishing RC input if we ever got a valid input */
		if (_rc_last_valid == 0) {
			/* we have never seen valid RC signals, abort */
			return OK;
		}
	}

	/* lazily advertise on first publication */
	if (_to_input_rc == nullptr) {
		_to_input_rc = orb_advertise(ORB_ID(input_rc), &rc_val);

	} else {
		orb_publish(ORB_ID(input_rc), _to_input_rc, &rc_val);  //来自io的rc输入
	}

	return OK;
}

int
PX4IO::io_publish_pwm_outputs()  //io发布pwm输出
{
	/* data we are going to fetch */
	actuator_outputs_s outputs = {};
	multirotor_motor_limits_s motor_limits;  //马达限制

	outputs.timestamp = hrt_absolute_time();  //输出时间戳

	/* get servo values from IO */  //获取来自io口的伺服值
	uint16_t ctl[_max_actuators];
	int ret = io_reg_get(PX4IO_PAGE_SERVOS, 0, ctl, _max_actuators);

	if (ret != OK) {
		return ret;
	}

	/* convert from register format to float */  //将寄存器格式转换为浮点
	for (unsigned i = 0; i < _max_actuators; i++) {
		outputs.output[i] = ctl[i];
	}

	outputs.noutputs = _max_actuators;

	/* lazily advertise on first publication */
	if (_to_outputs == nullptr) {
		int instance;
		_to_outputs = orb_advertise_multi(ORB_ID(actuator_outputs),
						  &outputs, &instance, ORB_PRIO_MAX);

	} else {
		orb_publish(ORB_ID(actuator_outputs), _to_outputs, &outputs);
	}

	/* get mixer status flags from IO */  //获取来自io的混控器状态
	uint16_t mixer_status;
	ret = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_MIXER, &mixer_status, sizeof(mixer_status) / sizeof(uint16_t));
	memcpy(&motor_limits, &mixer_status, sizeof(motor_limits));

	if (ret != OK) {
		return ret;
	}

	/* publish mixer status */  //发布混控器状态
	if (_to_mixer_status == nullptr) {
		_to_mixer_status = orb_advertise(ORB_ID(multirotor_motor_limits), &motor_limits);

	} else {
		orb_publish(ORB_ID(multirotor_motor_limits), _to_mixer_status, &motor_limits);
	}

	return OK;
}

int
PX4IO::io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)  //设置io板的寄存器
{
	/* range check the transfer */  //发送量程检查
	if (num_values > ((_max_transfer) / sizeof(*values))) {  //数量检查
		DEVICE_DEBUG("io_reg_set: too many registers (%u, max %u)", num_values, _max_transfer / 2);
		return -EINVAL;
	}

	int ret =  _interface->write((page << 8) | offset, (void *)values, num_values);

	if (ret != (int)num_values) {
		DEVICE_DEBUG("io_reg_set(%u,%u,%u): error %d", page, offset, num_values, ret);
		return -1;
	}

	return OK;
}

int
PX4IO::io_reg_set(uint8_t page, uint8_t offset, uint16_t value)
{
	return io_reg_set(page, offset, &value, 1);
}

int
PX4IO::io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values)  //获取io控制板的寄存器数据
{
	/* range check the transfer */
	if (num_values > ((_max_transfer) / sizeof(*values))) {  //数量检查
		DEVICE_DEBUG("io_reg_get: too many registers (%u, max %u)", num_values, _max_transfer / 2);
		return -EINVAL;
	}

	int ret = _interface->read((page << 8) | offset, reinterpret_cast<void *>(values), num_values);

	if (ret != (int)num_values) {
		DEVICE_DEBUG("io_reg_get(%u,%u,%u): data error %d", page, offset, num_values, ret);
		return -1;
	}

	return OK;
}

uint32_t
PX4IO::io_reg_get(uint8_t page, uint8_t offset)  //读取寄存器
{
	uint16_t value;

	if (io_reg_get(page, offset, &value, 1) != OK) {
		return _io_reg_get_error;
	}

	return value;
}

int
PX4IO::io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits)  //修改寄存器
{
	int ret;
	uint16_t value;

	ret = io_reg_get(page, offset, &value, 1); //先读取

	if (ret != OK) {
		return ret;
	}

	value &= ~clearbits; //修改值
	value |= setbits;

	return io_reg_set(page, offset, value); //写入寄存器
}

int
PX4IO::print_debug()  //打印debug信息
{
	int io_fd = -1;

	if (io_fd <= 0) {
		io_fd = ::open("/dev/ttyS0", O_RDONLY | O_NONBLOCK | O_NOCTTY);  //打开io设备
	}

	/* read IO's output */ //读取io输出
	if (io_fd >= 0) {  //设备打开成功
		pollfd fds[1];
		fds[0].fd = io_fd;
		fds[0].events = POLLIN;

		usleep(500);
		int pret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), 0);  //轮询

		if (pret > 0) { //轮询成功
			int count;
			char buf[65];

			do {
				count = ::read(io_fd, buf, sizeof(buf) - 1);  //读取io口

				if (count > 0) {
					/* enforce null termination */
					buf[count] = '\0';
					warnx("IO CONSOLE: %s", buf);
				}

			} while (count > 0);
		}

		::close(io_fd); //关闭设备
		return 0;
	}

	return 1;

}

int
PX4IO::mixer_send(const char *buf, unsigned buflen, unsigned retries)  //混控发送
{
	/* get debug level */
	int debuglevel = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SET_DEBUG);  //获取debug等级

	uint8_t	frame[_max_transfer];  //最大传送

	do {

		px4io_mixdata *msg = (px4io_mixdata *)&frame[0];  //混控数据
		unsigned max_len = _max_transfer - sizeof(px4io_mixdata);

		msg->f2i_mixer_magic = F2I_MIXER_MAGIC;
		msg->action = F2I_MIXER_ACTION_RESET;

		do {
			unsigned count = buflen;

			if (count > max_len) { //长度限制
				count = max_len;
			}

			if (count > 0) {
				memcpy(&msg->text[0], buf, count);
				buf += count;
				buflen -= count;

			} else {
				continue;
			}

			/*
			 * We have to send an even number of bytes.  This
			 * will only happen on the very last transfer of a
			 * mixer, and we are guaranteed that there will be
			 * space left to round up as _max_transfer will be
			 * even.  我们不得不发送偶数数量的字节，
			 *
			 */
			unsigned total_len = sizeof(px4io_mixdata) + count;  //总混控数据长度

			if (total_len % 2) {
				msg->text[count] = '\0';
				total_len++;
			}

			int ret;

			for (int i = 0; i < 30; i++) {
				/* failed, but give it a 2nd shot */
				ret = io_reg_set(PX4IO_PAGE_MIXERLOAD, 0, (uint16_t *)frame, total_len / 2);

				if (ret) {
					usleep(333);

				} else {
					break;
				}
			}

			/* print mixer chunk */  //打印混控器块
			if (debuglevel > 5 || ret) {

				warnx("fmu sent: \"%s\"", msg->text);

				/* read IO's output */
				print_debug();
			}

			if (ret) {
				DEVICE_LOG("mixer send error %d", ret);
				return ret;
			}

			msg->action = F2I_MIXER_ACTION_APPEND;

		} while (buflen > 0);

		int ret;

		/* send the closing newline */
		msg->text[0] = '\n';
		msg->text[1] = '\0';

		for (int i = 0; i < 30; i++) {
			/* failed, but give it a 2nd shot */
			ret = io_reg_set(PX4IO_PAGE_MIXERLOAD, 0, (uint16_t *)frame, (sizeof(px4io_mixdata) + 2) / 2);

			if (ret) {
				usleep(333);

			} else {
				break;
			}
		}

		if (ret == 0) {
			/* success, exit */
			break;
		}

		retries--;

	} while (retries > 0);

	if (retries == 0) {
		mavlink_and_console_log_info(_mavlink_fd, "[IO] mixer upload fail");
		/* load must have failed for some reason */
		return -EINVAL;

	} else {
		/* all went well, set the mixer ok flag */
		return io_reg_modify(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, 0, PX4IO_P_STATUS_FLAGS_MIXER_OK);
	}
}

void
PX4IO::print_status(bool extended_status)  //打印状态
{
	/* basic configuration */  //基本配置
	printf("protocol %u hardware %u bootloader %u buffer %uB crc 0x%04x%04x\n",
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_HARDWARE_VERSION),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_BOOTLOADER_VERSION),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_MAX_TRANSFER),
	       io_reg_get(PX4IO_PAGE_SETUP,  PX4IO_P_SETUP_CRC),
	       io_reg_get(PX4IO_PAGE_SETUP,  PX4IO_P_SETUP_CRC + 1));
	printf("%u controls %u actuators %u R/C inputs %u analog inputs %u relays\n",
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_CONTROL_COUNT),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_ACTUATOR_COUNT),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RC_INPUT_COUNT),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_ADC_INPUT_COUNT),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RELAY_COUNT));

	/* status */  //状态
	printf("%u bytes free\n",
	       io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FREEMEM));
	uint16_t flags = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS);
	uint16_t io_status_flags = flags;
	printf("status 0x%04x%s%s%s%s%s%s%s%s%s%s%s%s%s%s\n",
	       flags,
	       ((flags & PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED) ? " OUTPUTS_ARMED" : ""),
	       ((flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) ? " SAFETY_OFF" : " SAFETY_SAFE"),
	       ((flags & PX4IO_P_STATUS_FLAGS_OVERRIDE) ? " OVERRIDE" : ""),
	       ((flags & PX4IO_P_STATUS_FLAGS_RC_OK)    ? " RC_OK" : " RC_FAIL"),
	       ((flags & PX4IO_P_STATUS_FLAGS_RC_PPM)   ? " PPM" : ""),
	       ((flags & PX4IO_P_STATUS_FLAGS_RC_DSM)   ? " DSM" : ""),
	       ((flags & PX4IO_P_STATUS_FLAGS_RC_ST24)   ? " ST24" : ""),
	       ((flags & PX4IO_P_STATUS_FLAGS_RC_SBUS)  ? " SBUS" : ""),
	       ((flags & PX4IO_P_STATUS_FLAGS_FMU_OK)   ? " FMU_OK" : " FMU_FAIL"),
	       ((flags & PX4IO_P_STATUS_FLAGS_RAW_PWM)  ? " RAW_PWM_PASSTHROUGH" : ""),
	       ((flags & PX4IO_P_STATUS_FLAGS_MIXER_OK) ? " MIXER_OK" : " MIXER_FAIL"),
	       ((flags & PX4IO_P_STATUS_FLAGS_ARM_SYNC) ? " ARM_SYNC" : " ARM_NO_SYNC"),
	       ((flags & PX4IO_P_STATUS_FLAGS_INIT_OK)  ? " INIT_OK" : " INIT_FAIL"),
	       ((flags & PX4IO_P_STATUS_FLAGS_FAILSAFE)  ? " FAILSAFE" : ""));
	uint16_t alarms = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS);
	printf("alarms 0x%04x%s%s%s%s%s%s%s%s\n",
	       alarms,
	       ((alarms & PX4IO_P_STATUS_ALARMS_VBATT_LOW)     ? " VBATT_LOW" : ""),
	       ((alarms & PX4IO_P_STATUS_ALARMS_TEMPERATURE)   ? " TEMPERATURE" : ""),
	       ((alarms & PX4IO_P_STATUS_ALARMS_SERVO_CURRENT) ? " SERVO_CURRENT" : ""),
	       ((alarms & PX4IO_P_STATUS_ALARMS_ACC_CURRENT)   ? " ACC_CURRENT" : ""),
	       ((alarms & PX4IO_P_STATUS_ALARMS_FMU_LOST)      ? " FMU_LOST" : ""),
	       ((alarms & PX4IO_P_STATUS_ALARMS_RC_LOST)       ? " RC_LOST" : ""),
	       ((alarms & PX4IO_P_STATUS_ALARMS_PWM_ERROR)     ? " PWM_ERROR" : ""),
	       ((alarms & PX4IO_P_STATUS_ALARMS_VSERVO_FAULT)  ? " VSERVO_FAULT" : ""));
	/* now clear alarms */  //清除警告
	io_reg_set(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS, 0xFFFF);

	if (_hardware == 1) {
		printf("vbatt mV %u ibatt mV %u vbatt scale %u\n",
		       io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_VBATT),
		       io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_IBATT),
		       io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_VBATT_SCALE));
		printf("amp_per_volt %.3f amp_offset %.3f mAh discharged %.3f\n",
		       (double)_battery_amp_per_volt,
		       (double)_battery_amp_bias,
		       (double)_battery_mamphour_total);

	} else if (_hardware == 2) {
		printf("vservo %u mV vservo scale %u\n",
		       io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_VSERVO),
		       io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_VSERVO_SCALE));
		printf("vrssi %u\n", io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_VRSSI));
	}

	printf("actuators");  //执行机构

	for (unsigned i = 0; i < _max_actuators; i++) {
		printf(" %hi", int16_t(io_reg_get(PX4IO_PAGE_ACTUATORS, i)));
	}

	printf("\n");
	printf("servos");

	for (unsigned i = 0; i < _max_actuators; i++) {
		printf(" %u", io_reg_get(PX4IO_PAGE_SERVOS, i));
	}

	uint16_t pwm_invert_mask = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_REVERSE);

	printf("\n");
	printf("reversed outputs: [");

	for (unsigned i = 0; i < _max_actuators; i++) {
		printf("%s", (pwm_invert_mask & (1 << i)) ? "x" : "_");
	}

	printf("]");

	float trim_roll = REG_TO_FLOAT(io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_TRIM_ROLL));
	float trim_pitch = REG_TO_FLOAT(io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_TRIM_PITCH));
	float trim_yaw = REG_TO_FLOAT(io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_TRIM_YAW));

	printf(" trims: r: %8.4f p: %8.4f y: %8.4f\n",
	       (double)trim_roll, (double)trim_pitch, (double)trim_yaw);

	uint16_t raw_inputs = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT);
	printf("%d raw R/C inputs", raw_inputs);

	for (unsigned i = 0; i < raw_inputs; i++) {
		printf(" %u", io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE + i));
	}

	printf("\n");

	flags = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_FLAGS);
	printf("R/C flags: 0x%04x%s%s%s%s%s\n", flags,
	       (((io_status_flags & PX4IO_P_STATUS_FLAGS_RC_DSM) && (!(flags & PX4IO_P_RAW_RC_FLAGS_RC_DSM11))) ? " DSM10" : ""),
	       (((io_status_flags & PX4IO_P_STATUS_FLAGS_RC_DSM) && (flags & PX4IO_P_RAW_RC_FLAGS_RC_DSM11)) ? " DSM11" : ""),
	       ((flags & PX4IO_P_RAW_RC_FLAGS_FRAME_DROP) ? " FRAME_DROP" : ""),
	       ((flags & PX4IO_P_RAW_RC_FLAGS_FAILSAFE) ? " FAILSAFE" : ""),
	       ((flags & PX4IO_P_RAW_RC_FLAGS_MAPPING_OK) ? " MAPPING_OK" : "")
	      );

	if ((io_status_flags & PX4IO_P_STATUS_FLAGS_RC_PPM)) {
		int frame_len = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_DATA);
		printf("RC data (PPM frame len) %u us\n", frame_len);

		if ((frame_len - raw_inputs * 2000 - 3000) < 0) {
			printf("WARNING  WARNING  WARNING! This RC receiver does not allow safe frame detection.\n");
		}
	}

	uint16_t mapped_inputs = io_reg_get(PX4IO_PAGE_RC_INPUT, PX4IO_P_RC_VALID);
	printf("mapped R/C inputs 0x%04x", mapped_inputs);

	for (unsigned i = 0; i < _max_rc_input; i++) {
		if (mapped_inputs & (1 << i)) {
			printf(" %u:%d", i, REG_TO_SIGNED(io_reg_get(PX4IO_PAGE_RC_INPUT, PX4IO_P_RC_BASE + i)));
		}
	}

	printf("\n");
	uint16_t adc_inputs = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_ADC_INPUT_COUNT);
	printf("ADC inputs");

	for (unsigned i = 0; i < adc_inputs; i++) {
		printf(" %u", io_reg_get(PX4IO_PAGE_RAW_ADC_INPUT, i));
	}

	printf("\n");

	/* setup and state */
	uint16_t features = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES);
	printf("features 0x%04x%s%s%s%s\n", features,
	       ((features & PX4IO_P_SETUP_FEATURES_SBUS1_OUT) ? " S.BUS1_OUT" : ""),
	       ((features & PX4IO_P_SETUP_FEATURES_SBUS2_OUT) ? " S.BUS2_OUT" : ""),
	       ((features & PX4IO_P_SETUP_FEATURES_PWM_RSSI) ? " RSSI_PWM" : ""),
	       ((features & PX4IO_P_SETUP_FEATURES_ADC_RSSI) ? " RSSI_ADC" : "")
	      );
	uint16_t arming = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING);
	printf("arming 0x%04x%s%s%s%s%s%s%s%s%s%s\n",
	       arming,
	       ((arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)		? " FMU_ARMED" : " FMU_DISARMED"),
	       ((arming & PX4IO_P_SETUP_ARMING_IO_ARM_OK)		? " IO_ARM_OK" : " IO_ARM_DENIED"),
	       ((arming & PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK)	? " MANUAL_OVERRIDE_OK" : ""),
	       ((arming & PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM)		? " FAILSAFE_CUSTOM" : ""),
	       ((arming & PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK)	? " INAIR_RESTART_OK" : ""),
	       ((arming & PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE)	? " ALWAYS_PWM_ENABLE" : ""),
	       ((arming & PX4IO_P_SETUP_ARMING_LOCKDOWN)		? " LOCKDOWN" : ""),
	       ((arming & PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE)		? " FORCE_FAILSAFE" : ""),
	       ((arming & PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE) ? " TERM_FAILSAFE" : ""),
	       ((arming & PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE) ? " OVERRIDE_IMMEDIATE" : "")
	      );

	printf("rates 0x%04x default %u alt %u\n",
	       io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_RATES),
	       io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_DEFAULTRATE),
	       io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_ALTRATE));

	printf("debuglevel %u\n", io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SET_DEBUG));

	for (unsigned group = 0; group < 4; group++) {
		printf("controls %u:", group);

		for (unsigned i = 0; i < _max_controls; i++) {
			printf(" %d", (int16_t) io_reg_get(PX4IO_PAGE_CONTROLS, group * PX4IO_PROTOCOL_MAX_CONTROL_COUNT + i));
		}

		printf("\n");
	}

	if (extended_status) {
		for (unsigned i = 0; i < _max_rc_input; i++) {
			unsigned base = PX4IO_P_RC_CONFIG_STRIDE * i;
			uint16_t options = io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_OPTIONS);
			printf("input %u min %u center %u max %u deadzone %u assigned %u options 0x%04x%s%s\n",
			       i,
			       io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_MIN),
			       io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_CENTER),
			       io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_MAX),
			       io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_DEADZONE),
			       io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_ASSIGNMENT),
			       options,
			       ((options & PX4IO_P_RC_CONFIG_OPTIONS_ENABLED) ? " ENABLED" : ""),
			       ((options & PX4IO_P_RC_CONFIG_OPTIONS_REVERSE) ? " REVERSED" : ""));
		}
	}

	printf("failsafe");

	for (unsigned i = 0; i < _max_actuators; i++) {
		printf(" %u", io_reg_get(PX4IO_PAGE_FAILSAFE_PWM, i));
	}

	printf("\ndisarmed values");

	for (unsigned i = 0; i < _max_actuators; i++) {
		printf(" %u", io_reg_get(PX4IO_PAGE_DISARMED_PWM, i));
	}

	printf("\n");
}

int
PX4IO::ioctl(file *filep, int cmd, unsigned long arg)
{
	int ret = OK;

	/* regular ioctl? */
	switch (cmd) {
	case PWM_SERVO_ARM: //伺服解锁
		/* set the 'armed' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_FMU_ARMED);  //修改寄存器
		break;

	case PWM_SERVO_SET_ARM_OK:  //设置解锁
		/* set the 'OK to arm' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_IO_ARM_OK);
		break;

	case PWM_SERVO_CLEAR_ARM_OK:  //清除解锁
		/* clear the 'OK to arm' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_IO_ARM_OK, 0);
		break;

	case PWM_SERVO_DISARM:  //伺服加锁
		/* clear the 'armed' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_FMU_ARMED, 0);
		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:  //获取默认的更新速率
		/* get the default update rate */
		*(unsigned *)arg = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_DEFAULTRATE);
		break;

	case PWM_SERVO_SET_UPDATE_RATE:  //设置伺服更新速率
		/* set the requested alternate rate */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_ALTRATE, arg);
		break;

	case PWM_SERVO_GET_UPDATE_RATE:   //获取伺服更新速率
		/* get the alternative update rate */
		*(unsigned *)arg = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_ALTRATE);
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE: { //设置选择更新速率

			/* blindly clear the PWM update alarm - might be set for some other reason */
			io_reg_set(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS, PX4IO_P_STATUS_ALARMS_PWM_ERROR);

			/* attempt to set the rate map */
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_RATES, arg);

			/* check that the changes took */
			uint16_t alarms = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS);

			if (alarms & PX4IO_P_STATUS_ALARMS_PWM_ERROR) {
				ret = -EINVAL;
				io_reg_set(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS, PX4IO_P_STATUS_ALARMS_PWM_ERROR);
			}

			break;
		}

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:  //获取选择的更新速率

		*(unsigned *)arg = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_RATES);
		break;

	case PWM_SERVO_SET_FAILSAFE_PWM: {  //设置失效保护pwm
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			ret = io_reg_set(PX4IO_PAGE_FAILSAFE_PWM, 0, pwm->values, pwm->channel_count);
			break;
		}

	case PWM_SERVO_GET_FAILSAFE_PWM: //获取失效保护pwm

		ret = io_reg_get(PX4IO_PAGE_FAILSAFE_PWM, 0, (uint16_t *)arg, _max_actuators);

		if (ret != OK) {
			ret = -EIO;
		}

		break;

	case PWM_SERVO_SET_DISARMED_PWM: {  //设置加锁pwm
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			ret = io_reg_set(PX4IO_PAGE_DISARMED_PWM, 0, pwm->values, pwm->channel_count);
			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM:  //获取加锁pwm

		ret = io_reg_get(PX4IO_PAGE_DISARMED_PWM, 0, (uint16_t *)arg, _max_actuators);

		if (ret != OK) {
			ret = -EIO;
		}

		break;

	case PWM_SERVO_SET_MIN_PWM: { //设置伺服最小pwm
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			ret = io_reg_set(PX4IO_PAGE_CONTROL_MIN_PWM, 0, pwm->values, pwm->channel_count);
			break;
		}

	case PWM_SERVO_GET_MIN_PWM:  //获取伺服最小pwm值

		ret = io_reg_get(PX4IO_PAGE_CONTROL_MIN_PWM, 0, (uint16_t *)arg, _max_actuators);

		if (ret != OK) {
			ret = -EIO;
		}

		break;

	case PWM_SERVO_SET_MAX_PWM: { //设置伺服最大pwm
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			ret = io_reg_set(PX4IO_PAGE_CONTROL_MAX_PWM, 0, pwm->values, pwm->channel_count);
			break;
		}

	case PWM_SERVO_GET_MAX_PWM: //获取伺服最大pwm

		ret = io_reg_get(PX4IO_PAGE_CONTROL_MAX_PWM, 0, (uint16_t *)arg, _max_actuators);

		if (ret != OK) {
			ret = -EIO;
		}

		break;

	case PWM_SERVO_GET_COUNT: //获取伺服数量
		*(unsigned *)arg = _max_actuators;
		break;

	case PWM_SERVO_SET_DISABLE_LOCKDOWN:  //设置伺服禁止锁定
		_lockdown_override = arg;
		break;

	case PWM_SERVO_GET_DISABLE_LOCKDOWN: //获取伺服禁止锁定
		*(unsigned *)arg = _lockdown_override;
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_OFF:  //强制安全开关关闭
		/* force safety swith off */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_OFF, PX4IO_FORCE_SAFETY_MAGIC);
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_ON:  //强制安全开关打开
		/* force safety switch on */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_ON, PX4IO_FORCE_SAFETY_MAGIC);
		break;

	case PWM_SERVO_SET_FORCE_FAILSAFE: //强制四小保护

		/* force failsafe mode instantly */
		if (arg == 0) {
			/* clear force failsafe flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE, 0);

		} else {
			/* set force failsafe flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE);
		}

		break;

	case PWM_SERVO_SET_TERMINATION_FAILSAFE:  //设置伺服终止失效保护

		/* if failsafe occurs, do not allow the system to recover */
		if (arg == 0) {
			/* clear termination failsafe flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE, 0);

		} else {
			/* set termination failsafe flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE);
		}

		break;

	case PWM_SERVO_SET_OVERRIDE_IMMEDIATE: //立即覆盖fmu失效

		/* control whether override on FMU failure is
		   immediate or waits for override threshold on mode
		   switch */
		if (arg == 0) {
			/* clear override immediate flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE, 0);

		} else {
			/* set override immediate flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE);
		}

		break;

	case DSM_BIND_START:  //dsm绑定

		/* only allow DSM2, DSM-X and DSM-X with more than 7 channels */
		if (arg == DSM2_BIND_PULSES ||
		    arg == DSMX_BIND_PULSES ||
		    arg == DSMX8_BIND_PULSES) {
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_power_down);
			usleep(500000);
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_set_rx_out);
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_power_up);
			usleep(72000);
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_send_pulses | (arg << 4));
			usleep(50000);
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_reinit_uart);

			ret = OK;

		} else {
			ret = -EINVAL;
		}

		break;

	case DSM_BIND_POWER_UP:
		io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_power_up);
		break;

	case PWM_SERVO_SET(0) ... PWM_SERVO_SET(PWM_OUTPUT_MAX_CHANNELS - 1): {

			/* TODO: we could go lower for e.g. TurboPWM */
			unsigned channel = cmd - PWM_SERVO_SET(0);

			if ((channel >= _max_actuators) || (arg < PWM_LOWEST_MIN) || (arg > PWM_HIGHEST_MAX)) {
				ret = -EINVAL;

			} else {
				/* send a direct PWM value */
				ret = io_reg_set(PX4IO_PAGE_DIRECT_PWM, channel, arg);
			}

			break;
		}

	case PWM_SERVO_GET(0) ... PWM_SERVO_GET(PWM_OUTPUT_MAX_CHANNELS - 1): {

			unsigned channel = cmd - PWM_SERVO_GET(0);

			if (channel >= _max_actuators) {
				ret = -EINVAL;

			} else {
				/* fetch a current PWM value */
				uint32_t value = io_reg_get(PX4IO_PAGE_SERVOS, channel);

				if (value == _io_reg_get_error) {
					ret = -EIO;

				} else {
					*(servo_position_t *)arg = value;
				}
			}

			break;
		}

	case PWM_SERVO_GET_RATEGROUP(0) ... PWM_SERVO_GET_RATEGROUP(PWM_OUTPUT_MAX_CHANNELS - 1): {

			unsigned channel = cmd - PWM_SERVO_GET_RATEGROUP(0);

			*(uint32_t *)arg = io_reg_get(PX4IO_PAGE_PWM_INFO, PX4IO_RATE_MAP_BASE + channel);

			if (*(uint32_t *)arg == _io_reg_get_error) {
				ret = -EIO;
			}

			break;
		}

	case GPIO_RESET: { //gpio设置

			ret = -EINVAL;
			break;
		}

	case GPIO_SET:

		ret = -EINVAL;
		break;

	case GPIO_CLEAR:

		ret = -EINVAL;
		break;

	case GPIO_GET:

		ret = -EINVAL;
		break;

	case MIXERIOCGETOUTPUTCOUNT:  //混控器io获取输出数量
		*(unsigned *)arg = _max_actuators;
		break;

	case MIXERIOCRESET:
		ret = 0;	/* load always resets */
		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			ret = mixer_send(buf, strnlen(buf, 2048));
			break;
		}

	case RC_INPUT_GET: { //获取rc输入
			uint16_t status;
			rc_input_values *rc_val = (rc_input_values *)arg;

			ret = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, &status, 1);

			if (ret != OK) {
				break;
			}

			/* if no R/C input, don't try to fetch anything */
			if (!(status & PX4IO_P_STATUS_FLAGS_RC_OK)) {
				ret = -ENOTCONN;
				break;
			}

			/* sort out the source of the values */
			if (status & PX4IO_P_STATUS_FLAGS_RC_PPM) {
				rc_val->input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM;

			} else if (status & PX4IO_P_STATUS_FLAGS_RC_DSM) {
				rc_val->input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SPEKTRUM;

			} else if (status & PX4IO_P_STATUS_FLAGS_RC_SBUS) {
				rc_val->input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SBUS;

			} else if (status & PX4IO_P_STATUS_FLAGS_RC_ST24) {
				rc_val->input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_ST24;

			} else {
				rc_val->input_source = input_rc_s::RC_INPUT_SOURCE_UNKNOWN;
			}

			/* read raw R/C input values */
			ret = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE, &(rc_val->values[0]), _max_rc_input);
			break;
		}

	case PX4IO_SET_DEBUG:  //设置debug
		/* set the debug level */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SET_DEBUG, arg);
		break;

	case PX4IO_REBOOT_BOOTLOADER: //复位io部分到bootloader
		if (system_status() & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) {
			return -EINVAL;
		}

		/* reboot into bootloader - arg must be PX4IO_REBOOT_BL_MAGIC */
		io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_REBOOT_BL, arg);
		// we don't expect a reply from this operation
		ret = OK;
		break;

	case PX4IO_CHECK_CRC: { //检查io部分固件crc
			/* check IO firmware CRC against passed value */
			uint32_t io_crc = 0;
			ret = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_CRC, (uint16_t *)&io_crc, 2);

			if (ret != OK) {
				return ret;
			}

			if (io_crc != arg) {
				DEVICE_DEBUG("crc mismatch 0x%08x 0x%08x", (unsigned)io_crc, arg);
				return -EINVAL;
			}

			break;
		}

	case PX4IO_INAIR_RESTART_ENABLE:  //使能在空中复位

		/* set/clear the 'in-air restart' bit */
		if (arg) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK);

		} else {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK, 0);
		}

		break;

	case RC_INPUT_ENABLE_RSSI_ANALOG:

		if (arg) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_ADC_RSSI);

		} else {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, PX4IO_P_SETUP_FEATURES_ADC_RSSI, 0);
		}

		break;

	case RC_INPUT_ENABLE_RSSI_PWM:

		if (arg) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_PWM_RSSI);

		} else {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, PX4IO_P_SETUP_FEATURES_PWM_RSSI, 0);
		}

		break;

	case SBUS_SET_PROTO_VERSION:  //sbus协议版本

		if (arg == 1) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_SBUS1_OUT);

		} else if (arg == 2) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_SBUS2_OUT);

		} else {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES,
					    (PX4IO_P_SETUP_FEATURES_SBUS1_OUT | PX4IO_P_SETUP_FEATURES_SBUS2_OUT), 0);
		}

		break;

	case PWM_SERVO_SET_RC_CONFIG: {  //设置rc配置
			/* enable setting of RC configuration without relying
			   on param_get()
			*/
			struct pwm_output_rc_config *config = (struct pwm_output_rc_config *)arg;

			if (config->channel >= input_rc_s::RC_INPUT_MAX_CHANNELS) {
				/* fail with error */
				return -E2BIG;
			}

			/* copy values to registers in IO */
			uint16_t regs[PX4IO_P_RC_CONFIG_STRIDE];
			uint16_t offset = config->channel * PX4IO_P_RC_CONFIG_STRIDE;
			regs[PX4IO_P_RC_CONFIG_MIN]        = config->rc_min;
			regs[PX4IO_P_RC_CONFIG_CENTER]     = config->rc_trim;
			regs[PX4IO_P_RC_CONFIG_MAX]        = config->rc_max;
			regs[PX4IO_P_RC_CONFIG_DEADZONE]   = config->rc_dz;
			regs[PX4IO_P_RC_CONFIG_ASSIGNMENT] = config->rc_assignment;
			regs[PX4IO_P_RC_CONFIG_OPTIONS]    = PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;

			if (config->rc_reverse) {
				regs[PX4IO_P_RC_CONFIG_OPTIONS] |= PX4IO_P_RC_CONFIG_OPTIONS_REVERSE;
			}

			ret = io_reg_set(PX4IO_PAGE_RC_CONFIG, offset, regs, PX4IO_P_RC_CONFIG_STRIDE);
			break;
		}

	case PWM_SERVO_SET_OVERRIDE_OK:  //设置覆盖
		/* set the 'OVERRIDE OK' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK);
		break;

	case PWM_SERVO_CLEAR_OVERRIDE_OK:  //清除覆盖标志
		/* clear the 'OVERRIDE OK' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK, 0);
		break;

	default:
		/* see if the parent class can make any use of it */
		ret = CDev::ioctl(filep, cmd, arg);
		break;
	}

	return ret;
}

ssize_t
PX4IO::write(file * /*filp*/, const char *buffer, size_t len)
/* Make it obvious that file * isn't used here */
{
	unsigned count = len / 2;

	if (count > _max_actuators) {  //执行机构数量限制
		count = _max_actuators;
	}

	if (count > 0) {

		perf_begin(_perf_write);
		int ret = io_reg_set(PX4IO_PAGE_DIRECT_PWM, 0, (uint16_t *)buffer, count);
		perf_end(_perf_write);

		if (ret != OK) {
			return ret;
		}
	}

	return count * 2;
}

int
PX4IO::set_update_rate(int rate)  //设置更新速率
{
	int interval_ms = 1000 / rate;

	if (interval_ms < UPDATE_INTERVAL_MIN) {
		interval_ms = UPDATE_INTERVAL_MIN;
		warnx("update rate too high, limiting interval to %d ms (%d Hz).", interval_ms, 1000 / interval_ms);
	}

	if (interval_ms > 100) {
		interval_ms = 100;
		warnx("update rate too low, limiting to %d ms (%d Hz).", interval_ms, 1000 / interval_ms);
	}

	_update_interval = interval_ms;
	return 0;
}

void
PX4IO::set_battery_current_scaling(float amp_per_volt, float amp_bias)  //设置电池当前缩放
{
	_battery_amp_per_volt = amp_per_volt;  //缩放值
	_battery_amp_bias = amp_bias;  //偏差值
}

extern "C" __EXPORT int px4io_main(int argc, char *argv[]);

namespace  //匿名名字空间
{

device::Device *
get_interface()  //获取接口，串口或者i2c
{
	device::Device *interface = nullptr;

#ifndef CONFIG_ARCH_BOARD_PX4FMU_V1

#ifdef PX4IO_SERIAL_BASE
	interface = PX4IO_serial_interface();
#endif

	if (interface != nullptr) {
		goto got;
	}

#endif

#ifdef PX4_I2C_OBDEV_PX4IO
	interface = PX4IO_i2c_interface();
#endif

	if (interface != nullptr) {
		goto got;
	}

	errx(1, "cannot alloc interface");

got:

	if (interface->init() != OK) {  //接口初始化
		delete interface;
		errx(1, "interface init failed");
	}

	return interface;
}

void
start(int argc, char *argv[])  //开始
{
	if (g_dev != nullptr) {
		errx(0, "already loaded"); //已经载入
	}

	/* allocate the interface */
	device::Device *interface = get_interface(); //首先获取通信接口

	/* create the driver - it will set g_dev */  //创建驱动
	(void)new PX4IO(interface);

	if (g_dev == nullptr) {
		delete interface;
		errx(1, "driver alloc failed");
	}

	bool rc_handling_disabled = false;

	/* disable RC handling on request */
	if (argc > 1) {
		if (!strcmp(argv[1], "norc")) { //no rc禁止rc

			rc_handling_disabled = true;

		} else {
			warnx("unknown argument: %s", argv[1]);
		}
	}

	if (OK != g_dev->init(rc_handling_disabled)) {
		delete g_dev;
		g_dev = nullptr;
		errx(1, "driver init failed");
	}

	exit(0);
}

void
detect(int argc, char *argv[])  //探测
{
	if (g_dev != nullptr) {
		errx(0, "already loaded");
	}

	/* allocate the interface */
	device::Device *interface = get_interface();  //分配接口

	/* create the driver - it will set g_dev */
	(void)new PX4IO(interface);

	if (g_dev == nullptr) {
		errx(1, "driver alloc failed");
	}

	int ret = g_dev->detect();  //设备探测

	delete g_dev;
	g_dev = nullptr;

	if (ret) {
		/* nonzero, error */
		exit(1);

	} else {
		exit(0);
	}
}

void
checkcrc(int argc, char *argv[])  //检查crc
{
	bool keep_running = false;

	if (g_dev == nullptr) {
		/* allocate the interface */
		device::Device *interface = get_interface();

		/* create the driver - it will set g_dev */
		(void)new PX4IO(interface);

		if (g_dev == nullptr) {
			errx(1, "driver alloc failed");
		}

	} else {
		/* its already running, don't kill the driver */
		keep_running = true;
	}

	/*
	  check IO CRC against CRC of a file
	 */
	if (argc < 2) {
		warnx("usage: px4io checkcrc filename");
		exit(1);
	}

	int fd = open(argv[1], O_RDONLY);  //打开文件

	if (fd == -1) {
		warnx("open of %s failed: %d", argv[1], errno);
		exit(1);
	}

	const uint32_t app_size_max = 0xf000;
	uint32_t fw_crc = 0;
	uint32_t nbytes = 0;

	while (true) {
		uint8_t buf[16];
		int n = read(fd, buf, sizeof(buf));

		if (n <= 0) { break; }

		fw_crc = crc32part(buf, n, fw_crc);  //进行crc计算
		nbytes += n;
	}

	close(fd);

	while (nbytes < app_size_max) {
		uint8_t b = 0xff;
		fw_crc = crc32part(&b, 1, fw_crc);
		nbytes++;
	}

	int ret = g_dev->ioctl(nullptr, PX4IO_CHECK_CRC, fw_crc);

	if (!keep_running) {
		delete g_dev;
		g_dev = nullptr;
	}

	if (ret != OK) {
		warn("check CRC failed: %d", ret);
		exit(1);
	}

	warnx("CRCs match");
	exit(0);
}

void
bind(int argc, char *argv[])  //绑定外部输入信号
{
	int pulses;

	if (g_dev == nullptr) {
		errx(1, "px4io must be started first");
	}

	if (argc < 3) {
		errx(0, "needs argument, use dsm2, dsmx or dsmx8");
	}

	if (!strcmp(argv[2], "dsm2")) {
		pulses = DSM2_BIND_PULSES;

	} else if (!strcmp(argv[2], "dsmx")) {
		pulses = DSMX_BIND_PULSES;

	} else if (!strcmp(argv[2], "dsmx8")) {
		pulses = DSMX8_BIND_PULSES;

	} else {
		errx(1, "unknown parameter %s, use dsm2, dsmx or dsmx8", argv[2]);
	}

	// Test for custom pulse parameter
	if (argc > 3) {
		pulses = atoi(argv[3]);
	}

	if (g_dev->system_status() & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) {
		errx(1, "system must not be armed");
	}

	g_dev->ioctl(nullptr, DSM_BIND_START, pulses);

	exit(0);

}

void
test(void)  //测试
{
	int		fd;
	unsigned	servo_count = 0;
	unsigned	pwm_value = 1000;
	int		direction = 1;
	int		ret;

	fd = open(PX4IO_DEVICE_PATH, O_WRONLY);  //打开io设备

	if (fd < 0) {
		err(1, "failed to open device");
	}

	if (ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count)) { //获取伺服数量
		err(1, "failed to get servo count");
	}

	/* tell IO that its ok to disable its safety with the switch */
	ret = ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);  //告诉io可以使用安全开关

	if (ret != OK) {
		err(1, "PWM_SERVO_SET_ARM_OK");
	}

	if (ioctl(fd, PWM_SERVO_ARM, 0)) { //解锁io
		err(1, "failed to arm servos");
	}

	struct pollfd fds;

	fds.fd = 0; /* stdin */

	fds.events = POLLIN;

	warnx("Press CTRL-C or 'c' to abort.");

	for (;;) {

		/* sweep all servos between 1000..2000 */  //扫除所有在1000-2000之间的伺服
		servo_position_t servos[servo_count];

		for (unsigned i = 0; i < servo_count; i++) {  //所有开着的伺服值设为1000
			servos[i] = pwm_value;
		}

		ret = write(fd, servos, sizeof(servos));  //写入io

		if (ret != (int)sizeof(servos)) {
			err(1, "error writing PWM servo data, wrote %u got %d", sizeof(servos), ret);
		}

		if (direction > 0) {  //方向
			if (pwm_value < 2000) {
				pwm_value++;

			} else {
				direction = -1;
			}

		} else {
			if (pwm_value > 1000) {
				pwm_value--;

			} else {
				direction = 1;
			}
		}

		/* readback servo values */  //读会伺服值
		for (unsigned i = 0; i < servo_count; i++) {
			servo_position_t value;

			if (ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&value)) {
				err(1, "error reading PWM servo %d", i);
			}

			if (value != servos[i]) {
				errx(1, "servo %d readback error, got %u expected %u", i, value, servos[i]);
			}
		}

		/* Check if user wants to quit */
		char c;
		ret = poll(&fds, 1, 0);

		if (ret > 0) {

			read(0, &c, 1);

			if (c == 0x03 || c == 0x63 || c == 'q') {
				warnx("User abort\n");
				exit(0);
			}
		}
	}
}

void
monitor(void)  //显示器
{
	/* clear screen */
	printf("\033[2J");  //清除屏幕

	unsigned cancels = 2;

	for (;;) {
		pollfd fds[1];  //

		fds[0].fd = 0;
		fds[0].events = POLLIN;

		if (poll(fds, 1, 2000) < 0) { //轮询
			errx(1, "poll fail");
		}

		if (fds[0].revents == POLLIN) {
			/* control logic is to cancel with any key */ //任意件取消控制逻辑
			char c;
			(void)read(0, &c, 1);  //读取

			if (cancels-- == 0) {
				printf("\033[2J\033[H"); /* move cursor home and clear screen */
				exit(0);
			}
		}

		if (g_dev != nullptr) {

			printf("\033[2J\033[H"); /* move cursor home and clear screen */
			(void)g_dev->print_status(false);
			(void)g_dev->print_debug();
			printf("\n\n\n[ Use 'px4io debug <N>' for more output. Hit <enter> three times to exit monitor mode ]\n");

		} else {
			errx(1, "driver not loaded, exiting");
		}
	}
}

void
if_test(unsigned mode)  //是否测试
{
	device::Device *interface = get_interface();  //获取接口
	int result;

	if (interface) {
		result = interface->ioctl(1, mode); /* XXX magic numbers */
		delete interface;

	} else {
		errx(1, "interface not loaded, exiting");
	}

	errx(0, "test returned %d", result);
}

void
lockdown(int argc, char *argv[])  //锁定
{
	if (g_dev != nullptr) {

		if (argc > 2 && !strcmp(argv[2], "disable")) {  //失能

			warnx("WARNING: ACTUATORS WILL BE LIVE IN HIL! PROCEED?");
			warnx("Press 'y' to enable, any other key to abort.");

			/* check if user wants to abort */
			char c;

			struct pollfd fds;
			int ret;
			hrt_abstime start = hrt_absolute_time();  //获取绝对时间
			const unsigned long timeout = 5000000;

			while (hrt_elapsed_time(&start) < timeout) {
				fds.fd = 0; /* stdin */
				fds.events = POLLIN;
				ret = poll(&fds, 1, 0);

				if (ret > 0) {

					if (read(0, &c, 1) > 0) {

						if (c != 'y') {
							exit(0);

						} else if (c == 'y') {
							break;
						}
					}
				}

				usleep(10000);
			}

			if (hrt_elapsed_time(&start) > timeout) {  //消耗时间
				errx(1, "TIMEOUT! ABORTED WITHOUT CHANGES.");
			}

			(void)g_dev->ioctl(0, PWM_SERVO_SET_DISABLE_LOCKDOWN, 1);

			warnx("WARNING: ACTUATORS ARE NOW LIVE IN HIL!");

		} else {
			(void)g_dev->ioctl(0, PWM_SERVO_SET_DISABLE_LOCKDOWN, 0);
			warnx("ACTUATORS ARE NOW SAFE IN HIL.");
		}

	} else {
		errx(1, "driver not loaded, exiting");
	}

	exit(0);
}

} /* namespace */

int
px4io_main(int argc, char *argv[])  //主
{
	/* check for sufficient number of arguments */  //检查足够数量的参数
	if (argc < 2) {
		goto out;
	}

	if (!strcmp(argv[1], "start")) {  //开始
		start(argc - 1, argv + 1);
	}

	if (!strcmp(argv[1], "detect")) {  //检测
		detect(argc - 1, argv + 1);
	}

	if (!strcmp(argv[1], "checkcrc")) {  //检查crc
		checkcrc(argc - 1, argv + 1);
	}

	if (!strcmp(argv[1], "update")) {  //更新

		if (g_dev != nullptr) {
			warnx("loaded, detaching first");
			/* stop the driver */
			delete g_dev;
			g_dev = nullptr;
		}

		PX4IO_Uploader *up;
		const char *fn[4];

		/* work out what we're uploading... */
		if (argc > 2) { //
			fn[0] = argv[2];
			fn[1] = nullptr;

		} else {

 defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
			fn[0] = "/etc/extras/px4io-v2.bin";
			fn[1] =	"/fs/microsd/px4io2.bin";
			fn[2] =	"/fs/microsd/px4io.bin";
			fn[3] =	nullptr;
		}

		up = new PX4IO_Uploader;  //更新固件
		int ret = up->upload(&fn[0]);
		delete up;

		switch (ret) {
		case OK:
			break;

		case -ENOENT:
			errx(1, "PX4IO firmware file not found");  //固件没有找到

		case -EEXIST:
		case -EIO:
			errx(1, "error updating PX4IO - check that bootloader mode is enabled");  //检查bootloader模式已经使能

		case -EINVAL:
			errx(1, "verify failed - retry the update"); //校正失败，重新更新

		case -ETIMEDOUT:
			errx(1, "timed out waiting for bootloader - power-cycle and try again");

		default:
			errx(1, "unexpected error %d", ret);
		}

		return ret;
	}

	if (!strcmp(argv[1], "iftest")) {
		if (g_dev != nullptr) {
			errx(1, "can't iftest when started");
		}

		if_test((argc > 2) ? strtol(argv[2], NULL, 0) : 0);
	}

	if (!strcmp(argv[1], "forceupdate")) {   //强制更新
		/*
		  force update of the IO firmware without requiring  强制更新io固件不需要请求保持安全开关按下
		  the user to hold the safety switch down
		 */
		if (argc <= 3) {
			warnx("usage: px4io forceupdate MAGIC filename");
			exit(1);
		}

		if (g_dev == nullptr) {
			warnx("px4io is not started, still attempting upgrade");

			/* allocate the interface */
			device::Device *interface = get_interface();

			/* create the driver - it will set g_dev */
			(void)new PX4IO(interface);

			if (g_dev == nullptr) {
				delete interface;
				errx(1, "driver alloc failed");
			}
		}

		uint16_t arg = atol(argv[2]);
		int ret = g_dev->ioctl(nullptr, PX4IO_REBOOT_BOOTLOADER, arg);

		if (ret != OK) {
			warnx("reboot failed - %d", ret);
			exit(1);
		}

		// tear down the px4io instance  取消接口
		delete g_dev;
		g_dev = nullptr;

		// upload the specified firmware  上传指定固件
		const char *fn[2];
		fn[0] = argv[3];
		fn[1] = nullptr;
		PX4IO_Uploader *up = new PX4IO_Uploader;
		up->upload(&fn[0]);
		delete up;
		exit(0);
	}

	/* commands below here require a started driver */  //以下命令需要启动驱动

	if (g_dev == nullptr) {
		errx(1, "not started");
	}

	if (!strcmp(argv[1], "limit")) {  //限制

		if ((argc > 2)) {
			int limitrate = atoi(argv[2]);

			if (limitrate > 0) {
				g_dev->set_update_rate(limitrate);  //设置更新速率

			} else {
				errx(1, "invalid rate: %d", limitrate);
			}

		} else {
			errx(1, "missing argument (50 - 500 Hz)");
			return 1;
		}

		exit(0);
	}

	if (!strcmp(argv[1], "current")) {
		if ((argc > 3)) {
			g_dev->set_battery_current_scaling(atof(argv[2]), atof(argv[3]));

		} else {
			errx(1, "missing argument (apm_per_volt, amp_offset)");
			return 1;
		}

		exit(0);
	}

	if (!strcmp(argv[1], "safety_off")) {  //安全开关关闭
		int ret = g_dev->ioctl(NULL, PWM_SERVO_SET_FORCE_SAFETY_OFF, 0);

		if (ret != OK) {
			warnx("failed to disable safety");
			exit(1);
		}

		exit(0);
	}

	if (!strcmp(argv[1], "safety_on")) {  //安全开关打开
		int ret = g_dev->ioctl(NULL, PWM_SERVO_SET_FORCE_SAFETY_ON, 0);

		if (ret != OK) {
			warnx("failed to enable safety");
			exit(1);
		}

		exit(0);
	}

	if (!strcmp(argv[1], "recovery")) {  //使能空中复位

		/*
		 * Enable in-air restart support.
		 * We can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		g_dev->ioctl(NULL, PX4IO_INAIR_RESTART_ENABLE, 1);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) { //停止

		/* stop the driver */
		delete g_dev;
		g_dev = nullptr;
		exit(0);
	}


	if (!strcmp(argv[1], "status")) {  //状态

		warnx("loaded");
		g_dev->print_status(true);

		exit(0);
	}

	if (!strcmp(argv[1], "debug")) {  //debug
		if (argc <= 2) {
			warnx("usage: px4io debug LEVEL");
			exit(1);
		}

		if (g_dev == nullptr) {
			warnx("not started");
			exit(1);
		}

		uint8_t level = atoi(argv[2]);
		/* we can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		int ret = g_dev->ioctl(nullptr, PX4IO_SET_DEBUG, level);

		if (ret != 0) {
			warnx("SET_DEBUG failed: %d", ret);
			exit(1);
		}

		warnx("SET_DEBUG %u OK", (unsigned)level);
		exit(0);
	}

	if (!strcmp(argv[1], "rx_dsm") ||
	    !strcmp(argv[1], "rx_dsm_10bit") ||
	    !strcmp(argv[1], "rx_dsm_11bit") ||
	    !strcmp(argv[1], "rx_sbus") ||
	    !strcmp(argv[1], "rx_ppm")) {
		errx(0, "receiver type is automatically detected, option '%s' is deprecated", argv[1]);
	}

	if (!strcmp(argv[1], "test")) {  //测试
		test();
	}

	if (!strcmp(argv[1], "monitor")) {  //显示
		monitor();
	}

	if (!strcmp(argv[1], "bind")) {
		bind(argc, argv);
	}

	if (!strcmp(argv[1], "lockdown")) {
		lockdown(argc, argv);
	}

	if (!strcmp(argv[1], "sbus1_out")) {
		/* we can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		int ret = g_dev->ioctl(nullptr, SBUS_SET_PROTO_VERSION, 1);

		if (ret != 0) {
			errx(ret, "S.BUS v1 failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "sbus2_out")) {
		/* we can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		int ret = g_dev->ioctl(nullptr, SBUS_SET_PROTO_VERSION, 2);

		if (ret != 0) {
			errx(ret, "S.BUS v2 failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "rssi_analog")) {
		/* we can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		int ret = g_dev->ioctl(nullptr, RC_INPUT_ENABLE_RSSI_ANALOG, 1);

		if (ret != 0) {
			errx(ret, "RSSI analog failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "rssi_pwm")) {
		/* we can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		int ret = g_dev->ioctl(nullptr, RC_INPUT_ENABLE_RSSI_PWM, 1);

		if (ret != 0) {
			errx(ret, "RSSI PWM failed");
		}

		exit(0);
	}

out:
	errx(1, "need a command, try 'start', 'stop', 'status', 'test', 'monitor', 'debug <level>',\n"
	     "'recovery', 'limit <rate>', 'current', 'bind', 'checkcrc', 'safety_on', 'safety_off',\n"
	     "'forceupdate', 'update', 'sbus1_out', 'sbus2_out', 'rssi_analog' or 'rssi_pwm'");
}
