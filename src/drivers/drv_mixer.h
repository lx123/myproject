
/**
 * @file drv_mixer.h
 *
 * Mixer ioctl interfaces.
 *
 * Normal workflow is:  正常工作流程：
 *
 * - open mixer device  打开混控器设备
 * - add mixer(s)  //添加混控
 * loop:
 *  - mix actuators to array  混合执行机构到数组
 *
 * Each client has its own configuration.  每个客户端都有自己的配置
 *
 * When mixing, outputs are produced by mixers in the order they are  当混合的时候，混控器产生输出，为了他们被添加。
 * added.  A simple mixer produces one output; a multirotor mixer will  一个简单的混控器产生一个输出，一个多旋翼混控器将产生几个输出
 * produce several outputs, etc.
 */

#ifndef _DRV_MIXER_H
#define _DRV_MIXER_H

#include <px4_defines.h>
#include <stdint.h>
#include <sys/ioctl.h>

#define MIXER0_DEVICE_PATH		"/dev/mixer0"

/*
 * ioctl() definitions
 */
#define _MIXERIOCBASE		(0x2500)
#define _MIXERIOC(_n)		(_PX4_IOC(_MIXERIOCBASE, _n))

/** get the number of mixable outputs */
#define MIXERIOCGETOUTPUTCOUNT	_MIXERIOC(0)  //获取混口那个起输出数量

/** reset (clear) the mixer configuration */
#define MIXERIOCRESET		_MIXERIOC(1)  //复位/清除混控器配置

/** simple channel scaler */  //简单通道缩放
struct mixer_scaler_s {
	float			negative_scale;  //负缩放
	float			positive_scale;  //正缩放
	float			offset;   //偏移
	float			min_output;  //最小暑促
	float			max_output;  //最大输出
};

/** mixer input */  //混控输入
struct mixer_control_s {
	uint8_t			control_group;	/**< group from which the input reads */
	uint8_t			control_index;	/**< index within the control group */
	struct mixer_scaler_s 	scaler;		/**< scaling applied to the input before use */
};

/** simple mixer */
struct mixer_simple_s {
	uint8_t			control_count;	/**< number of inputs */
	struct mixer_scaler_s	output_scaler;	/**< scaling for the output */
	struct mixer_control_s	controls[0];	/**< actual size of the array is set by control_count */
};

#define MIXER_SIMPLE_SIZE(_icount)	(sizeof(struct mixer_simple_s) + (_icount) * sizeof(struct mixer_control_s))

/**
 * add a simple mixer in (struct mixer_simple_s *)arg  添加一个简单的混控
 */
#define MIXERIOCADDSIMPLE	_MIXERIOC(2)

/* _MIXERIOC(3) was deprecated */
/* _MIXERIOC(4) was deprecated */

/**
 * Add mixer(s) from the buffer in (const char *)arg
 */
#define MIXERIOCLOADBUF		_MIXERIOC(5)

/*
 * XXX Thoughts for additional operations:
 *
 * - get/set output scale, for tuning center/limit values.
 * - save/serialise for saving tuned mixers.
 */

#endif /* _DRV_ACCEL_H */
