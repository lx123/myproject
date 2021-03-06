
/**
 * @file mixer.cpp
 *
 * Control channel input/output mixer and failsafe.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <nuttx/config.h>
#include <syslog.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>
#include <rc/sbus.h>

#include <systemlib/pwm_limit/pwm_limit.h>
#include <systemlib/mixer/mixer.h>
#include <uORB/topics/actuator_controls.h>  //执行机构控制

extern "C" {
//#define DEBUG
#include "io.h"
}

/*
 * Maximum interval in us before FMU signal is considered lost
 */
#define FMU_INPUT_DROP_LIMIT_US		500000  //认为fmu失效的最大时间间隔
#define NAN_VALUE	(0.0f/0.0f)   //无穷大数据

/* current servo arm/disarm state */   //当前伺服加锁和解锁状态
static bool mixer_servos_armed = false;  //混合器伺服解锁
static bool should_arm = false;
static bool should_arm_nothrottle = false;
static bool should_always_enable_pwm = false;  //始终使能pwm
static volatile bool in_mixer = false;

extern int _sbus_fd;  //sbus文件句柄

/* selected control values and count for mixing */
enum mixer_source {  //混控器源头
	MIX_NONE,
	MIX_FMU,
	MIX_OVERRIDE,
	MIX_FAILSAFE,
	MIX_OVERRIDE_FMU_OK
};
static mixer_source source;

static int	mixer_callback(uintptr_t handle,
			       uint8_t control_group,
			       uint8_t control_index,
			       float &control);  //回调

static MixerGroup mixer_group(mixer_callback, 0);

void
mixer_tick(void)   //混控滴答
{

	/* check that we are receiving fresh data from the FMU */  //检查是否有新数据来自fmu的
	if ((system_state.fmu_data_received_time == 0) ||
	    hrt_elapsed_time(&system_state.fmu_data_received_time) > FMU_INPUT_DROP_LIMIT_US) {  //如果数据接收为0或者两次时间间隔大于500ms

		/* too long without FMU input, time to go to failsafe */  //太长时间没有收到fmu输入，定时器进入失效保护
		if (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK) {  //上一次fmu状态ok
			isr_debug(1, "AP RX timeout");  //接收超时
		}

		r_status_flags &= ~(PX4IO_P_STATUS_FLAGS_FMU_OK);   //ok标志为清零
		r_status_alarms |= PX4IO_P_STATUS_ALARMS_FMU_LOST;  //fmu失效警告标志置位

	} else {
		r_status_flags |= PX4IO_P_STATUS_FLAGS_FMU_OK;  //设置标志ok

		/* this flag is never cleared once OK */
		r_status_flags |= PX4IO_P_STATUS_FLAGS_FMU_INITIALIZED;  //fmu已经初始化，这个标志一旦置位，不会被清零
	}

	/* default to failsafe mixing - it will be forced below if flag is set */
	source = MIX_FAILSAFE;

	/*
	 * Decide which set of controls we're using.
	 */

	/* do not mix if RAW_PWM mode is on and FMU is good */  //
	if ((r_status_flags & PX4IO_P_STATUS_FLAGS_RAW_PWM) &&
	    (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK)) {

		/* don't actually mix anything - we already have raw PWM values */
		source = MIX_NONE;  //实际上不混合任何东西，我们已经有原始pwm值

	} else {

		if (!(r_status_flags & PX4IO_P_STATUS_FLAGS_OVERRIDE) &&  //手动覆盖
		    (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK) &&     //fmu ok
		    (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK)) {  //混控ok

			/* mix from FMU controls */
			source = MIX_FMU;   //来自fmu的混控
		}

		if ((r_status_flags & PX4IO_P_STATUS_FLAGS_OVERRIDE) &&  //手动覆盖
		    (r_status_flags & PX4IO_P_STATUS_FLAGS_RC_OK) &&     //rc输入
		    (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK) &&   //混控开关
		    !(r_setup_arming & PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED) &&  //
		    !(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK) &&
		    /* do not enter manual override if we asked for termination failsafe and FMU is lost */
		    !(r_setup_arming & PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE)) {

			/* if allowed, mix from RC inputs directly */
			source = MIX_OVERRIDE;  //如果允许，直接接收来自rc的控制

		} else 	if ((r_status_flags & PX4IO_P_STATUS_FLAGS_OVERRIDE) &&
			    (r_status_flags & PX4IO_P_STATUS_FLAGS_RC_OK) &&
			    (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK) &&
			    !(r_setup_arming & PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED) &&
			    (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK)) {

			/* if allowed, mix from RC inputs directly up to available rc channels */
			source = MIX_OVERRIDE_FMU_OK;  //如果允许，混控直接来自rc输入上升到rc可用通道
		}
	}

	/*
	 * Decide whether the servos should be armed right now.  立即决定那个伺服应该启动
	 *
	 * We must be armed, and we must have a PWM source; either raw from
	 * FMU or from the mixer.  我们必须解锁，我们必须有pwm源头，从fmu或混控器中来
	 *
	 */
	should_arm = (
			     /* IO initialised without error */ (r_status_flags & PX4IO_P_STATUS_FLAGS_INIT_OK)   //io初始化没有错误
			     /* and IO is armed */ 		  && (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)    //io解锁
			     /* and FMU is armed */ 		  && (
				     ((r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)    //fmu解锁
				      /* and there is valid input via or mixer */         && (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK))  //混控器有有效输入
				     /* or direct PWM is set */               || (r_status_flags & PX4IO_P_STATUS_FLAGS_RAW_PWM)   //或者直接pwm设置
				     /* or failsafe was set manually */	 || ((r_setup_arming & PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM)
						     && !(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK))
			     )
		     );

	should_arm_nothrottle = (
					/* IO initialised without error */ (r_status_flags & PX4IO_P_STATUS_FLAGS_INIT_OK)
					/* and IO is armed */ 		  && (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)
					/* and there is valid input via or mixer */         && (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK));

	should_always_enable_pwm = (r_setup_arming & PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE)
				   && (r_status_flags & PX4IO_P_STATUS_FLAGS_INIT_OK)
				   && (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK);

	/*
	 * Check if failsafe termination is set - if yes,
	 * set the force failsafe flag once entering the first
	 * failsafe condition.
	 */
	if ( /* if we have requested flight termination style failsafe (noreturn) */
		(r_setup_arming & PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE) &&
		/* and we ended up in a failsafe condition */
		(source == MIX_FAILSAFE) &&
		/* and we should be armed, so we intended to provide outputs */
		should_arm &&
		/* and FMU is initialized */
		(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_INITIALIZED)) {
		r_setup_arming |= PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE;
	}

	/*
	 * Check if we should force failsafe - and do it if we have to
	 */
	if (r_setup_arming & PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE) {
		source = MIX_FAILSAFE;
	}

	/*
	 * Set failsafe status flag depending on mixing source
	 */
	if (source == MIX_FAILSAFE) {
		r_status_flags |= PX4IO_P_STATUS_FLAGS_FAILSAFE;

	} else {
		r_status_flags &= ~(PX4IO_P_STATUS_FLAGS_FAILSAFE);
	}

	/*
	 * Run the mixers.
	 */
	if (source == MIX_FAILSAFE) {

		/* copy failsafe values to the servo outputs */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			r_page_servos[i] = r_page_servo_failsafe[i];

			/* safe actuators for FMU feedback */
			r_page_actuators[i] = FLOAT_TO_REG((r_page_servos[i] - 1500) / 600.0f);
		}


	} else if (source != MIX_NONE && (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK)
		   && !(r_setup_arming & PX4IO_P_SETUP_ARMING_LOCKDOWN)) {

		float	outputs[PX4IO_SERVO_COUNT];
		unsigned mixed;

		/* mix */

		/* poor mans mutex */
		in_mixer = true;
		mixed = mixer_group.mix(&outputs[0], PX4IO_SERVO_COUNT, &r_mixer_limits);
		in_mixer = false;

		/* the pwm limit call takes care of out of band errors */
		pwm_limit_calc(should_arm, should_arm_nothrottle, mixed, r_setup_pwm_reverse, r_page_servo_disarmed,
			       r_page_servo_control_min, r_page_servo_control_max, outputs, r_page_servos, &pwm_limit);

		/* clamp unused outputs to zero */
		for (unsigned i = mixed; i < PX4IO_SERVO_COUNT; i++) {
			r_page_servos[i] = 0;
			outputs[i] = 0.0f;
		}

		/* store normalized outputs */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			r_page_actuators[i] = FLOAT_TO_REG(outputs[i]);
		}
	}

	/* set arming */
	bool needs_to_arm = (should_arm || should_arm_nothrottle || should_always_enable_pwm);

	/* lockdown means to send a valid pulse which disables the outputs */
	if (r_setup_arming & PX4IO_P_SETUP_ARMING_LOCKDOWN) {
		needs_to_arm = true;
	}

	if (needs_to_arm && !mixer_servos_armed) {
		/* need to arm, but not armed */
		up_pwm_servo_arm(true);
		mixer_servos_armed = true;
		r_status_flags |= PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED;
		isr_debug(5, "> PWM enabled");

	} else if (!needs_to_arm && mixer_servos_armed) {
		/* armed but need to disarm */
		up_pwm_servo_arm(false);
		mixer_servos_armed = false;
		r_status_flags &= ~(PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED);
		isr_debug(5, "> PWM disabled");
	}

	if (mixer_servos_armed && (should_arm || should_arm_nothrottle)
	    && !(r_setup_arming & PX4IO_P_SETUP_ARMING_LOCKDOWN)) {
		/* update the servo outputs. */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			up_pwm_servo_set(i, r_page_servos[i]);
		}

		/* set S.BUS1 or S.BUS2 outputs */

		if (r_setup_features & PX4IO_P_SETUP_FEATURES_SBUS2_OUT) {
			sbus2_output(_sbus_fd, r_page_servos, PX4IO_SERVO_COUNT);

		} else if (r_setup_features & PX4IO_P_SETUP_FEATURES_SBUS1_OUT) {
			sbus1_output(_sbus_fd, r_page_servos, PX4IO_SERVO_COUNT);
		}

	} else if (mixer_servos_armed && (should_always_enable_pwm
					  || (r_setup_arming & PX4IO_P_SETUP_ARMING_LOCKDOWN))) {
		/* set the disarmed servo outputs. */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			up_pwm_servo_set(i, r_page_servo_disarmed[i]);
			/* copy values into reporting register */
			r_page_servos[i] = r_page_servo_disarmed[i];
		}

		/* set S.BUS1 or S.BUS2 outputs */
		if (r_setup_features & PX4IO_P_SETUP_FEATURES_SBUS1_OUT) {
			sbus1_output(_sbus_fd, r_page_servo_disarmed, PX4IO_SERVO_COUNT);
		}

		if (r_setup_features & PX4IO_P_SETUP_FEATURES_SBUS2_OUT) {
			sbus2_output(_sbus_fd, r_page_servo_disarmed, PX4IO_SERVO_COUNT);
		}
	}
}

static int
mixer_callback(uintptr_t handle,
	       uint8_t control_group,
	       uint8_t control_index,
	       float &control)
{
	if (control_group >= PX4IO_CONTROL_GROUPS) {
		return -1;
	}

	switch (source) {
	case MIX_FMU:
		if (control_index < PX4IO_CONTROL_CHANNELS && control_group < PX4IO_CONTROL_GROUPS) {
			control = REG_TO_FLOAT(r_page_controls[CONTROL_PAGE_INDEX(control_group, control_index)]);
			break;
		}

		return -1;

	case MIX_OVERRIDE:
		if (r_page_rc_input[PX4IO_P_RC_VALID] & (1 << CONTROL_PAGE_INDEX(control_group, control_index))) {
			control = REG_TO_FLOAT(r_page_rc_input[PX4IO_P_RC_BASE + control_index]);
			break;
		}

		return -1;

	case MIX_OVERRIDE_FMU_OK:

		/* FMU is ok but we are in override mode, use direct rc control for the available rc channels. The remaining channels are still controlled by the fmu */
		if (r_page_rc_input[PX4IO_P_RC_VALID] & (1 << CONTROL_PAGE_INDEX(control_group, control_index))) {
			control = REG_TO_FLOAT(r_page_rc_input[PX4IO_P_RC_BASE + control_index]);
			break;

		} else if (control_index < PX4IO_CONTROL_CHANNELS && control_group < PX4IO_CONTROL_GROUPS) {
			control = REG_TO_FLOAT(r_page_controls[CONTROL_PAGE_INDEX(control_group, control_index)]);
			break;
		}

		return -1;

	case MIX_FAILSAFE:
	case MIX_NONE:
		control = 0.0f;
		return -1;
	}

	/* apply trim offsets for override channels */
	if (source == MIX_OVERRIDE || source == MIX_OVERRIDE_FMU_OK) {
		if (control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
		    control_index == actuator_controls_s::INDEX_ROLL) {
			control += REG_TO_FLOAT(r_setup_trim_roll);

		} else if (control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
			   control_index == actuator_controls_s::INDEX_PITCH) {
			control += REG_TO_FLOAT(r_setup_trim_pitch);

		} else if (control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
			   control_index == actuator_controls_s::INDEX_YAW) {
			control += REG_TO_FLOAT(r_setup_trim_yaw);
		}
	}

	/* limit output */
	if (control > 1.0f) {
		control = 1.0f;

	} else if (control < -1.0f) {
		control = -1.0f;
	}

	/* motor spinup phase - lock throttle to zero */
	if ((pwm_limit.state == PWM_LIMIT_STATE_RAMP) || (should_arm_nothrottle && !should_arm)) {
		if (control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* limit the throttle output to zero during motor spinup,
			 * as the motors cannot follow any demand yet
			 */
			control = 0.0f;
		}
	}

	/* only safety off, but not armed - set throttle as invalid */
	if (should_arm_nothrottle && !should_arm) {
		if (control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* mark the throttle as invalid */
			control = NAN_VALUE;
		}
	}

	return 0;
}

/*
 * XXX error handling here should be more aggressive; currently it is
 * possible to get STATUS_FLAGS_MIXER_OK set even though the mixer has
 * not loaded faithfully.
 */

static char mixer_text[200];		/* large enough for one mixer */
static unsigned mixer_text_length = 0;

int
mixer_handle_text(const void *buffer, size_t length)  //
{
	/* do not allow a mixer change while safety off and FMU armed */  //如果安全关闭并且fmu解锁，不允许一个混控器改变
	if ((r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) &&
	    (r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)) {
		return 1;
	}

	/* disable mixing, will be enabled once load is complete */  //禁止混合，将会使能一次载入完成
	r_status_flags &= ~(PX4IO_P_STATUS_FLAGS_MIXER_OK);  //清除混控ok标志

	/* abort if we're in the mixer - the caller is expected to retry */
	if (in_mixer) {  //如果我们在混控
		return 1;
	}

	px4io_mixdata	*msg = (px4io_mixdata *)buffer;  //取出缓冲区数据

	isr_debug(2, "mix txt %u", length);  //输出混合文本长度

	if (length < sizeof(px4io_mixdata)) { //如果小于一帧数据
		return 0; //返回错误
	}

	unsigned text_length = length - sizeof(px4io_mixdata);  //计算文本长度

	switch (msg->action) {  //解析动作
	case F2I_MIXER_ACTION_RESET:  //复位混控器
		isr_debug(2, "reset");

		/* THEN actually delete it */
		mixer_group.reset();  //移除组内所有的混控器
		mixer_text_length = 0;  //混控器文本长度清零

	/* FALLTHROUGH */
	case F2I_MIXER_ACTION_APPEND:
		isr_debug(2, "append %d", length);

		/* check for overflow - this would be really fatal */  //检查移除，这将是致命错误
		if ((mixer_text_length + text_length + 1) > sizeof(mixer_text)) {
			r_status_flags &= ~PX4IO_P_STATUS_FLAGS_MIXER_OK;  //清除混控ok标志
			return 0;
		}

		/* append mixer text and nul-terminate, guard against overflow */
		memcpy(&mixer_text[mixer_text_length], msg->text, text_length);
		mixer_text_length += text_length;
		mixer_text[mixer_text_length] = '\0';
		isr_debug(2, "buflen %u", mixer_text_length); //混控器文本长度

		/* process the text buffer, adding new mixers as their descriptions can be parsed */ //处理文本缓冲区，添加新的混控器作为他们描述的解析
		unsigned resid = mixer_text_length;
		mixer_group.load_from_buf(&mixer_text[0], resid);  //从缓冲区中载入

		/* if anything was parsed */  //如果一些被解析
		if (resid != mixer_text_length) {

			isr_debug(2, "used %u", mixer_text_length - resid);

			/* copy any leftover text to the base of the buffer for re-use */
			if (resid > 0) {
				memcpy(&mixer_text[0], &mixer_text[mixer_text_length - resid], resid);
			}

			mixer_text_length = resid;
		}

		break;
	}

	return 0;
}

void
mixer_set_failsafe()  //设置失效保护
{
	/*
	 * Check if a custom failsafe value has been written,
	 * or if the mixer is not ok and bail out.
	 */

	if ((r_setup_arming & PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM) ||
	    !(r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK)) {
		return;
	}

	/* set failsafe defaults to the values for all inputs = 0 */
	float	outputs[PX4IO_SERVO_COUNT];
	unsigned mixed;

	/* mix */
	mixed = mixer_group.mix(&outputs[0], PX4IO_SERVO_COUNT, &r_mixer_limits);

	/* scale to PWM and update the servo outputs as required */
	for (unsigned i = 0; i < mixed; i++) {

		/* scale to servo output */
		r_page_servo_failsafe[i] = (outputs[i] * 600.0f) + 1500;

	}

	/* disable the rest of the outputs */
	for (unsigned i = mixed; i < PX4IO_SERVO_COUNT; i++) {
		r_page_servo_failsafe[i] = 0;
	}

}
