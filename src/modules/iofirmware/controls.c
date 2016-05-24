
/**
 * @file controls.c
 *
 * R/C inputs and servo outputs.
 */

#include <nuttx/config.h>
#include <stdbool.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>
#include <systemlib/perf_counter.h>
#include <systemlib/ppm_decode.h>
#include <rc/st24.h>
#include <rc/sumd.h>
#include <rc/sbus.h>
#include <rc/dsm.h>

#include "io.h"

#define RC_FAILSAFE_TIMEOUT		2000000		/**< two seconds failsafe timeout */   //失效保护超时时间2s
#define RC_CHANNEL_HIGH_THRESH		5000	/* 75% threshold */  //杆量
#define RC_CHANNEL_LOW_THRESH		-8000	/* 10% threshold */

static bool	ppm_input(uint16_t *values, uint16_t *num_values, uint16_t *frame_len);
static bool	dsm_port_input(uint16_t *rssi, bool *dsm_updated, bool *st24_updated, bool *sumd_updated);

static perf_counter_t c_gather_dsm;  //性能计数器
static perf_counter_t c_gather_sbus;
static perf_counter_t c_gather_ppm;

static int _dsm_fd = -1;  //文件句柄
int _sbus_fd = -1;

static uint16_t rc_value_override = 0;

#ifdef ADC_RSSI
static unsigned _rssi_adc_counts = 0;
#endif

bool dsm_port_input(uint16_t *rssi, bool *dsm_updated, bool *st24_updated, bool *sumd_updated)  //dsm口输入
{
	perf_begin(c_gather_dsm);
	uint8_t n_bytes = 0;
	uint8_t *bytes;
	bool dsm_11_bit;
	*dsm_updated = dsm_input(_dsm_fd, r_raw_rc_values, &r_raw_rc_count, &dsm_11_bit, &n_bytes, &bytes,
				 PX4IO_RC_INPUT_CHANNELS);

	if (*dsm_updated) {

		if (dsm_11_bit) {
			r_raw_rc_flags |= PX4IO_P_RAW_RC_FLAGS_RC_DSM11;

		} else {
			r_raw_rc_flags &= ~PX4IO_P_RAW_RC_FLAGS_RC_DSM11;
		}

		r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FRAME_DROP);
		r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FAILSAFE);

	}

	perf_end(c_gather_dsm);

	/* get data from FD and attempt to parse with DSM and ST24 libs */
	uint8_t st24_rssi, rx_count;
	uint16_t st24_channel_count = 0;

	*st24_updated = false;

	for (unsigned i = 0; i < n_bytes; i++) {
		/* set updated flag if one complete packet was parsed */
		st24_rssi = RC_INPUT_RSSI_MAX;
		*st24_updated |= (OK == st24_decode(bytes[i], &st24_rssi, &rx_count,
						    &st24_channel_count, r_raw_rc_values, PX4IO_RC_INPUT_CHANNELS));
	}

	if (*st24_updated) {

		/* ensure ADC RSSI is disabled */
		r_setup_features &= ~(PX4IO_P_SETUP_FEATURES_ADC_RSSI);

		*rssi = st24_rssi;
		r_raw_rc_count = st24_channel_count;

		r_status_flags |= PX4IO_P_STATUS_FLAGS_RC_ST24;
		r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FRAME_DROP);
		r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
	}


	/* get data from FD and attempt to parse with SUMD libs */
	uint8_t sumd_rssi, sumd_rx_count;
	uint16_t sumd_channel_count = 0;

	*sumd_updated = false;

	for (unsigned i = 0; i < n_bytes; i++) {
		/* set updated flag if one complete packet was parsed */
		sumd_rssi = RC_INPUT_RSSI_MAX;
		*sumd_updated |= (OK == sumd_decode(bytes[i], &sumd_rssi, &sumd_rx_count,
						    &sumd_channel_count, r_raw_rc_values, PX4IO_RC_INPUT_CHANNELS));
	}

	if (*sumd_updated) {

		/* not setting RSSI since SUMD does not provide one */
		r_raw_rc_count = sumd_channel_count;

		r_status_flags |= PX4IO_P_STATUS_FLAGS_RC_SUMD;
		r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FRAME_DROP);
		r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
	}

	return (*dsm_updated | *st24_updated | *sumd_updated);
}

void
controls_init(void)  //外部控制初始化
{
	/* no channels */
	r_raw_rc_count = 0;  //原始rc计数
	system_state.rc_channels_timestamp_received = 0;  //接收时间戳
	system_state.rc_channels_timestamp_valid = 0;   //有效时间戳

	/* DSM input (USART1) */
	_dsm_fd = dsm_init("/dev/ttyS0");

	/* S.bus input (USART3) */
	_sbus_fd = sbus_init("/dev/ttyS2", false);

	/* default to a 1:1 input map, all enabled */
    for (unsigned i = 0; i < PX4IO_RC_INPUT_CHANNELS; i++) {//8个输入通道
        unsigned base = PX4IO_P_RC_CONFIG_STRIDE * i;//控制通道起始位置，ppm协议

		r_page_rc_input_config[base + PX4IO_P_RC_CONFIG_OPTIONS]    = 0;
		r_page_rc_input_config[base + PX4IO_P_RC_CONFIG_MIN]        = 1000;
		r_page_rc_input_config[base + PX4IO_P_RC_CONFIG_CENTER]     = 1500;
		r_page_rc_input_config[base + PX4IO_P_RC_CONFIG_MAX]        = 2000;
		r_page_rc_input_config[base + PX4IO_P_RC_CONFIG_DEADZONE]   = 30;
		r_page_rc_input_config[base + PX4IO_P_RC_CONFIG_ASSIGNMENT] = i;
		r_page_rc_input_config[base + PX4IO_P_RC_CONFIG_OPTIONS]    = PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;
	}

    c_gather_dsm = perf_alloc(PC_ELAPSED, "c_gather_dsm");  //测量一个时间消耗的时间
	c_gather_sbus = perf_alloc(PC_ELAPSED, "c_gather_sbus");
	c_gather_ppm = perf_alloc(PC_ELAPSED, "c_gather_ppm");
}

void
controls_tick()  //控制计数
{

	/*
	 * Gather R/C control inputs from supported sources.  获取rc控制输入，来自支持的源
	 *
	 * Note that if you're silly enough to connect more than
	 * one control input source, they're going to fight each
	 * other.  Don't do that.
	 */

	/* receive signal strenght indicator (RSSI). 0 = no connection, 255: perfect connection */
	uint16_t rssi = 0;  //接收信号强度指示器

#ifdef ADC_RSSI  //模拟量

	if (r_setup_features & PX4IO_P_SETUP_FEATURES_ADC_RSSI) { //模拟量测量接收信号
		unsigned counts = adc_measure(ADC_RSSI);  //测量模拟值

		if (counts != 0xffff) {  //信号值不为最大
			/* low pass*/  //低通
			_rssi_adc_counts = (_rssi_adc_counts * 0.998f) + (counts * 0.002f);  //加权平均
			/* use 1:1 scaling on 3.3V, 12-Bit ADC input */
			unsigned mV = _rssi_adc_counts * 3300 / 4095;  //计算电压值
			/* scale to 0..100 (RC_INPUT_RSSI_MAX == 100) */
			rssi = (mV * RC_INPUT_RSSI_MAX / 3300);  //信号百分比

			if (rssi > RC_INPUT_RSSI_MAX) {  //限制100％
				rssi = RC_INPUT_RSSI_MAX;
			}
		}
	}

#endif

	/* zero RSSI if signal is lost */  //如果信号丢失，rssi接收信号强度为0
	if (!(r_raw_rc_flags & (PX4IO_P_RAW_RC_FLAGS_RC_OK))) {
		rssi = 0;
	}

	perf_begin(c_gather_dsm);  //获得dsm信号
	bool dsm_updated, st24_updated, sumd_updated;
	(void)dsm_port_input(&rssi, &dsm_updated, &st24_updated, &sumd_updated);

	if (dsm_updated) {  //如果dsm数据更新
		r_status_flags |= PX4IO_P_STATUS_FLAGS_RC_DSM;  //状态标志置位
	}

	if (st24_updated) {
		r_status_flags |= PX4IO_P_STATUS_FLAGS_RC_ST24;
	}

	if (sumd_updated) {
		r_status_flags |= PX4IO_P_STATUS_FLAGS_RC_SUMD;
	}

	perf_end(c_gather_dsm); //结束dsm性能计数

	perf_begin(c_gather_sbus);  //收集sbus信息

	bool sbus_failsafe, sbus_frame_drop;
	bool sbus_updated = sbus_input(_sbus_fd, r_raw_rc_values, &r_raw_rc_count, &sbus_failsafe, &sbus_frame_drop,
				       PX4IO_RC_INPUT_CHANNELS);

	if (sbus_updated) {  //如果sbus信息更新
		r_status_flags |= PX4IO_P_STATUS_FLAGS_RC_SBUS;  //标志位置位

		unsigned sbus_rssi = RC_INPUT_RSSI_MAX;

		if (sbus_frame_drop) {
			r_raw_rc_flags |= PX4IO_P_RAW_RC_FLAGS_FRAME_DROP;
			sbus_rssi = RC_INPUT_RSSI_MAX / 2;

		} else {
			r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FRAME_DROP);
		}

		if (sbus_failsafe) {
			r_raw_rc_flags |= PX4IO_P_RAW_RC_FLAGS_FAILSAFE;

		} else {
			r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
		}

		/* set RSSI to an emulated value if ADC RSSI is off */
		if (!(r_setup_features & PX4IO_P_SETUP_FEATURES_ADC_RSSI)) {
			rssi = sbus_rssi;
		}

	}

	perf_end(c_gather_sbus);

	/*
	 * XXX each S.bus frame will cause a PPM decoder interrupt
	 * storm (lots of edges).  It might be sensible to actually
	 * disable the PPM decoder completely if we have S.bus signal.
	 */
	perf_begin(c_gather_ppm);  //获取ppm数据
	bool ppm_updated = ppm_input(r_raw_rc_values, &r_raw_rc_count, &r_page_raw_rc_input[PX4IO_P_RAW_RC_DATA]);

	if (ppm_updated) { //ppm数据更新

		r_status_flags |= PX4IO_P_STATUS_FLAGS_RC_PPM;  //接收标志为ppm
		r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FRAME_DROP); //飞机掉落标志
		r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FAILSAFE);  //失效保护标志
	}

	perf_end(c_gather_ppm);

	/* limit number of channels to allowable data size */
	if (r_raw_rc_count > PX4IO_RC_INPUT_CHANNELS) { //限制ppm最大通道数
		r_raw_rc_count = PX4IO_RC_INPUT_CHANNELS;
	}

	/* store RSSI */
	r_page_raw_rc_input[PX4IO_P_RAW_RC_NRSSI] = rssi;  //保存接收信号强度指示器

	/*
	 * In some cases we may have received a frame, but input has still
	 * been lost.
	 */
	bool rc_input_lost = false;  //输入丢失标志为清零

	/*
	 * If we received a new frame from any of the RC sources, process it.
	 */
	if (dsm_updated || sbus_updated || ppm_updated || st24_updated || sumd_updated) {  //如果有任何一个rc获得信号

		/* record a bitmask of channels assigned */
		unsigned assigned_channels = 0;

		/* update RC-received timestamp */
		system_state.rc_channels_timestamp_received = hrt_absolute_time();  //更新接收时间戳

		/* update RC-received timestamp */
		system_state.rc_channels_timestamp_valid = system_state.rc_channels_timestamp_received; //跟新接收有效时间戳

		/* map raw inputs to mapped inputs */  //映射原始输入到映射的输入
		/* XXX mapping should be atomic relative to protocol */
		for (unsigned i = 0; i < r_raw_rc_count; i++) {  //rc原始输入数量，对每一个进行扫描

			/* map the input channel */
			uint16_t *conf = &r_page_rc_input_config[i * PX4IO_P_RC_CONFIG_STRIDE];  //映射输入通道

			if (conf[PX4IO_P_RC_CONFIG_OPTIONS] & PX4IO_P_RC_CONFIG_OPTIONS_ENABLED) {

				uint16_t raw = r_raw_rc_values[i];

				int16_t scaled;

				/*
				 * 1) Constrain to min/max values, as later processing depends on bounds.
				 */
				if (raw < conf[PX4IO_P_RC_CONFIG_MIN]) {  //限制最小值
					raw = conf[PX4IO_P_RC_CONFIG_MIN];
				}

				if (raw > conf[PX4IO_P_RC_CONFIG_MAX]) {  //限制最大值
					raw = conf[PX4IO_P_RC_CONFIG_MAX];
				}

				/*
				 * 2) Scale around the mid point differently for lower and upper range.
				 *
				 * This is necessary as they don't share the same endpoints and slope.
				 *
				 * First normalize to 0..1 range with correct sign (below or above center),
				 * then scale to 20000 range (if center is an actual center, -10000..10000,
				 * if parameters only support half range, scale to 10000 range, e.g. if
				 * center == min 0..10000, if center == max -10000..0).
				 *
				 * As the min and max bounds were enforced in step 1), division by zero
				 * cannot occur, as for the case of center == min or center == max the if
				 * statement is mutually exclusive with the arithmetic NaN case.
				 *
				 * DO NOT REMOVE OR ALTER STEP 1!
				 */
				if (raw > (conf[PX4IO_P_RC_CONFIG_CENTER] + conf[PX4IO_P_RC_CONFIG_DEADZONE])) {
					scaled = 10000.0f * ((raw - conf[PX4IO_P_RC_CONFIG_CENTER] - conf[PX4IO_P_RC_CONFIG_DEADZONE]) / (float)(
								     conf[PX4IO_P_RC_CONFIG_MAX] - conf[PX4IO_P_RC_CONFIG_CENTER] - conf[PX4IO_P_RC_CONFIG_DEADZONE]));

				} else if (raw < (conf[PX4IO_P_RC_CONFIG_CENTER] - conf[PX4IO_P_RC_CONFIG_DEADZONE])) {
					scaled = 10000.0f * ((raw - conf[PX4IO_P_RC_CONFIG_CENTER] + conf[PX4IO_P_RC_CONFIG_DEADZONE]) / (float)(
								     conf[PX4IO_P_RC_CONFIG_CENTER] - conf[PX4IO_P_RC_CONFIG_DEADZONE] - conf[PX4IO_P_RC_CONFIG_MIN]));

				} else {
					/* in the configured dead zone, output zero */  //如果在配置的死区内，输出0
					scaled = 0;
				}

				/* invert channel if requested */
				if (conf[PX4IO_P_RC_CONFIG_OPTIONS] & PX4IO_P_RC_CONFIG_OPTIONS_REVERSE) {  //如有请求，反转通道
					scaled = -scaled;
				}

				/* and update the scaled/mapped version */
				unsigned mapped = conf[PX4IO_P_RC_CONFIG_ASSIGNMENT];  //已经映射的通道

				if (mapped < PX4IO_CONTROL_CHANNELS) {  //在控制通道范围内

					/* invert channel if pitch - pulling the lever down means pitching up by convention */
					if (mapped == 1) {  //如果是pitch轴，反转
						/* roll, pitch, yaw, throttle, override is the standard order */
						scaled = -scaled;
					}

					if (mapped == 3 && r_setup_rc_thr_failsafe) {  //失效保护检测到
						/* throttle failsafe detection */
						if (((raw < conf[PX4IO_P_RC_CONFIG_MIN]) && (raw < r_setup_rc_thr_failsafe)) ||
						    ((raw > conf[PX4IO_P_RC_CONFIG_MAX]) && (raw > r_setup_rc_thr_failsafe))) {
							r_raw_rc_flags |= PX4IO_P_RAW_RC_FLAGS_FAILSAFE;

						} else {
							r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
						}
					}

					r_rc_values[mapped] = SIGNED_TO_REG(scaled);  //有符号数转换为无符号数
					assigned_channels |= (1 << mapped);

				} else if (mapped == PX4IO_P_RC_CONFIG_ASSIGNMENT_MODESWITCH) {
					/* pick out override channel, indicated by special mapping */
					rc_value_override = SIGNED_TO_REG(scaled);
				}
			}
		}

		/* set un-assigned controls to zero */  //设置为标记的控制为0
		for (unsigned i = 0; i < PX4IO_CONTROL_CHANNELS; i++) {
			if (!(assigned_channels & (1 << i))) {
				r_rc_values[i] = 0;
			}
		}

		/* set RC OK flag, as we got an update */
		r_status_flags |= PX4IO_P_STATUS_FLAGS_RC_OK;  //设置接收状态ok
		r_raw_rc_flags |= PX4IO_P_RAW_RC_FLAGS_RC_OK;  //设置原始rc数据ok

		/* if we have enough channels (5) to control the vehicle, the mapping is ok */
		if (assigned_channels > 4) {  //如果输入通道达到最小的5通道，通道映射ok
			r_raw_rc_flags |= PX4IO_P_RAW_RC_FLAGS_MAPPING_OK;

		} else {
			r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_MAPPING_OK);  //通道映射有问题
		}

		/*
		 * Export the valid channel bitmap 输出有效的通道绑定
		 */
		r_rc_valid = assigned_channels;
	}

	/*
	 * If we haven't seen any new control data in 200ms, assume we
	 * have lost input.  如果在200ms内没有收到控制数据，表示输入丢失
	 */
	if (hrt_elapsed_time(&system_state.rc_channels_timestamp_received) > 200000) {
		rc_input_lost = true; //rc输入丢失标志

		/* clear the input-kind flags here */
		r_status_flags &= ~(  //清除各种输入标志
					  PX4IO_P_STATUS_FLAGS_RC_PPM |
					  PX4IO_P_STATUS_FLAGS_RC_DSM |
					  PX4IO_P_STATUS_FLAGS_RC_SBUS);

	}

	/*
	 * Handle losing RC input
	 */

	/* if we are in failsafe, clear the override flag */
	if (r_raw_rc_flags & PX4IO_P_RAW_RC_FLAGS_FAILSAFE) {  //失效保护
		r_status_flags &= ~(PX4IO_P_STATUS_FLAGS_OVERRIDE);  //清除覆盖标志
	}

	/* this kicks in if the receiver is gone, but there is not on failsafe (indicated by separate flag) */
	if (rc_input_lost) {  //输入丢失
		/* Clear the RC input status flag, clear manual override flag */
		r_status_flags &= ~(  //清除rc输入状态标志，清除手动覆盖标志
					  PX4IO_P_STATUS_FLAGS_OVERRIDE |
					  PX4IO_P_STATUS_FLAGS_RC_OK);

		/* flag raw RC as lost */
		r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_RC_OK);  //标志，原始rc丢失

		/* Mark all channels as invalid, as we just lost the RX */
		r_rc_valid = 0;

		/* Set raw channel count to zero */
		r_raw_rc_count = 0;

		/* Set the RC_LOST alarm */
		r_status_alarms |= PX4IO_P_STATUS_ALARMS_RC_LOST;
	}

	/*
	 * Check for manual override.  检查手动覆盖
	 *
	 * The PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK flag must be set, and we
	 * must have R/C input (NO FAILSAFE!).
	 * Override is enabled if either the hardcoded channel / value combination
	 * is selected, or the AP has requested it.  如果硬件代码通道/值结合被选中或者ap请求了，覆盖被使能
	 */
	if ((!(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK) || (r_setup_arming & PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK)) &&
	    (r_status_flags & PX4IO_P_STATUS_FLAGS_RC_OK) &&
	    !(r_raw_rc_flags & PX4IO_P_RAW_RC_FLAGS_FAILSAFE)) {

		bool override = false;

		/*
		 * Check mapped channel 5 (can be any remote channel,  检查映射通道5
		 * depends on RC_MAP_OVER parameter);
		 * If the value is 'high' then the pilot has
		 * requested override.
		 *
		 */
		if ((r_status_flags & PX4IO_P_STATUS_FLAGS_RC_OK) && (REG_TO_SIGNED(rc_value_override) < RC_CHANNEL_LOW_THRESH)) {
			override = true;
		}

		/*
		  if the FMU is dead then enable override if we have a
		  mixer and OVERRIDE_IMMEDIATE is set
		 */
		if (!(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK) &&
		    (r_setup_arming & PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE) &&
		    (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK)) {
			override = true;
		}

		if (override) {

			r_status_flags |= PX4IO_P_STATUS_FLAGS_OVERRIDE;

			/* mix new RC input control values to servos */  //混合新的输入控制数据到伺服
			if (dsm_updated || sbus_updated || ppm_updated || st24_updated || sumd_updated) {
				mixer_tick();  //到混控mix
			}

		} else {
			r_status_flags &= ~(PX4IO_P_STATUS_FLAGS_OVERRIDE);
		}

	} else {
		r_status_flags &= ~(PX4IO_P_STATUS_FLAGS_OVERRIDE);
	}
}

static bool
ppm_input(uint16_t *values, uint16_t *num_values, uint16_t *frame_len)  //ppm输入
{
	bool result = false;

	if (!(num_values) || !(values) || !(frame_len)) {  //只要有一个参数为零就返回
		return result;
	}

	/* avoid racing with PPM updates */  //防止中断干扰
	irqstate_t state = up_irq_save();

	/*
	 * If we have received a new PPM frame within the last 200ms, accept it
	 * and then invalidate it.  如果我们接收到新的ppm框架在至多200ms内，接收它并使她失效
	 */
	if (hrt_elapsed_time(&ppm_last_valid_decode) < 200000) { //小于200ms接收

		/* PPM data exists, copy it */
		*num_values = ppm_decoded_channels;  //ppm解码通道熟练

		if (*num_values > PX4IO_RC_INPUT_CHANNELS) {  //限制最大通道
			*num_values = PX4IO_RC_INPUT_CHANNELS;
		}

		for (unsigned i = 0; ((i < *num_values) && (i < PPM_MAX_CHANNELS)); i++) {
			values[i] = ppm_buffer[i];  //获取各个通道数据
		}

		/* clear validity */
		ppm_last_valid_decode = 0;  //清楚有效标志

		/* store PPM frame length */
		*frame_len = ppm_frame_length; //保存ppm框架长度

		/* good if we got any channels */
		result = (*num_values > 0);
	}

	up_irq_restore(state);  //恢复中断

	return result;
}


