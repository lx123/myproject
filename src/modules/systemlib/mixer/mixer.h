
/**
 * @file mixer.h
 *
 * Generic, programmable, procedural control signal mixers. 通用，可编程，程序扣扣你告知信号混控器
 *
 * This library implements a generic mixer interface that can be used
 * by any driver or subsytem that wants to combine several control signals
 * into a single output. 结合几个控制信号到一个信号输出
 *
 * Terminology 术语
 * ===========
 *
 * control value 控制值
 *	A mixer input value, typically provided by some controlling
 *	component of the system. 混合输入值，由一些系统控制部分提供
 *
 * control group  控制组
 * 	A collection of controls provided by a single controlling component. 由单一的控制部分提供手指控制
 *
 * actuator 执行机构
 *	The mixer output value. 混合器输出值
 *
 *
 * Mixing basics
 * =============
 *
 * An actuator derives its value from the combination of one or more  一个执行机构的值来自一个或多个控制值的结合
 * control values. Each of the control values is scaled according to  每个控制值被缩放根据执行机构的配置然后结合构成执行值
 * the actuator's configuration and then combined to produce the
 * actuator value, which may then be further scaled to suit the specific  还可能被进一步缩放取适合特殊输出类型
 * output type.
 *
 * Internally, all scaling is performed using floating point values.  在内不，所有的缩放使用浮点值
 * Inputs and outputs are clamped to the range -1.0 to 1.0. 输入和输出强制在-1到1范围内
 *
 * control    control   control
 *    |          |         |
 *    v          v         v
 *  scale      scale     scale  缩放
 *    |          |         |
 *    |          v         |
 *    +-------> mix <------+    混合
 *               |
 *             scale   混合
 *               |
 *               v
 *              out  输出
 *
 * Scaling  缩放
 * -------
 *
 * Each scaler allows the input value to be scaled independently for  每个缩放器允许输入值被独立缩放，输入可以高于/低于0
 * inputs greater/less than zero. An offset can be applied to the output,  一个偏移合一应用到输出，同事页可以低于或高于边界约束
 * as well as lower and upper boundary constraints.
 * Negative scaling factors cause the output to be inverted (negative input  负数缩放因子倒置输出反向
 * produces positive output).
 *
 * Scaler pseudocode: 缩放伪代码
 *
 * if (input < 0)
 *     output = (input * NEGATIVE_SCALE) + OFFSET
 * else
 *     output = (input * POSITIVE_SCALE) + OFFSET
 *
 * if (output < LOWER_LIMIT)  //边界限制
 *     output = LOWER_LIMIT
 * if (output > UPPER_LIMIT)
 *     output = UPPER_LIMIT
 *
 *
 * Mixing  混合
 * ------
 *
 * Mixing behaviour varies based on the specific mixer class; each
 * mixer class describes its behaviour in more detail.
 *
 *
 * Controls  控制
 * --------
 *
 * The precise assignment of controls may vary depending on the
 * application, but the following assignments should be used
 * when appropriate.  Some mixer classes have specific assumptions
 * about the assignment of controls.
 *
 * control | standard meaning
 * --------+-----------------------
 *     0   | roll
 *     1   | pitch
 *     2   | yaw
 *     3   | primary thrust  主要推力
 */


#ifndef _SYSTEMLIB_MIXER_MIXER_H
#define _SYSTEMLIB_MIXER_MIXER_H value

#include <nuttx/config.h>
#include "drivers/drv_mixer.h"

#include <uORB/topics/multirotor_motor_limits.h>

#include "mixer_load.h"

/**
 * Abstract class defining a mixer mixing zero or more inputs to
 * one or more outputs. 抽象类定义一个混控器混合0或者更多输入到一个或多个输出
 */
class __EXPORT Mixer
{
public:
	/** next mixer in a list */
	Mixer				*_next;

	/**
	 * Fetch a control value.  获取一个控制值
	 *
	 * @param handle		Token passed when the callback is registered.
	 * @param control_group		The group to fetch the control from.
	 * @param control_index		The group-relative index to fetch the control from.
	 * @param control		The returned control
	 * @return			Zero if the value was fetched, nonzero otherwise.
	 */
	typedef int	(* ControlCallback)(uintptr_t handle,
					    uint8_t control_group,
					    uint8_t control_index,
					    float &control);

	/**
	 * Constructor.
	 *
	 * @param control_cb		Callback invoked when reading controls.
	 */
	Mixer(ControlCallback control_cb, uintptr_t cb_handle);  //构造函数
	virtual ~Mixer() {};

	/**
	 * Perform the mixing function.
	 *
	 * @param outputs		Array into which mixed output(s) should be placed.
	 * @param space			The number of available entries in the output array;
	 * @return			The number of entries in the output array that were populated.
	 */
	virtual unsigned		mix(float *outputs, unsigned space, uint16_t *status_reg) = 0;  //执行混控功能

	/**
	 * Analyses the mix configuration and updates a bitmask of groups  分析混合配置和更新组的位掩码
	 * that are required.
	 *
	 * @param groups		A bitmask of groups (0-31) that the mixer requires.
	 */
	virtual void			groups_required(uint32_t &groups) = 0;

protected:
	/** client-supplied callback used when fetching control values */  //客户端支持的回调，用来获取控制值
	ControlCallback			_control_cb;
	uintptr_t			_cb_handle;

	/**
	 * Invoke the client callback to fetch a control value.
	 *
	 * @param group			Control group to fetch from.
	 * @param index			Control index to fetch.
	 * @return			The control value.
	 */
	float				get_control(uint8_t group, uint8_t index);  //抢占客户端回调获取控制值

	/**
	 * Perform simpler linear scaling.
	 *
	 * @param scaler		The scaler configuration.
	 * @param input			The value to be scaled.
	 * @return			The scaled value.
	 */
	static float			scale(const mixer_scaler_s &scaler, float input);  //执行一个简单的线性缩放

	/**
	 * Validate a scaler
	 *
	 * @param scaler		The scaler to be validated.
	 * @return			Zero if good, nonzero otherwise.
	 */
	static int			scale_check(struct mixer_scaler_s &scaler);  //检查一个缩放

	/**
	 * Find a tag
	 *
	 * @param buf			The buffer to operate on.
	 * @param buflen		length of the buffer.
	 * @param tag			character to search for.
	 */
	static const char 		*findtag(const char *buf, unsigned &buflen, char tag);  //查找一个标签

	/**
	 * Skip a line
	 *
	 * @param buf			The buffer to operate on.
	 * @param buflen		length of the buffer.
	 * @return			0 / OK if a line could be skipped, 1 else
	 */
	static const char 		*skipline(const char *buf, unsigned &buflen);  //跳过一个行

private:

	/* do not allow to copy due to pointer data members */
	Mixer(const Mixer &);
	Mixer &operator=(const Mixer &);
};

/**
 * Group of mixers, built up from single mixers and processed
 * in order when mixing.  //混控器组
 */
class __EXPORT MixerGroup : public Mixer
{
public:
	MixerGroup(ControlCallback control_cb, uintptr_t cb_handle);
	~MixerGroup();

	virtual unsigned		mix(float *outputs, unsigned space, uint16_t *status_reg);
	virtual void			groups_required(uint32_t &groups);

	/**
	 * Add a mixer to the group.
	 *
	 * @param mixer			The mixer to be added.
	 */
	void				add_mixer(Mixer *mixer);  //添加一个混控器到组

	/**
	 * Remove all the mixers from the group.
	 */
	void				reset();  //移除组里所有的混控器

	/**
	 * Count the mixers in the group.
	 */
	unsigned			count();  //计算组里的混控器数量

	/**
	 * Adds mixers to the group based on a text description in a buffer.  添加一个混控器到组是基于一个缓冲区中文本描述
	 *
	 * Mixer definitions begin with a single capital letter and a colon.
	 * The actual format of the mixer definition varies with the individual
	 * mixers; they are summarised here, but see ROMFS/mixers/README for
	 * more details.
	 *
	 * Null Mixer
	 * ..........
	 *
	 * The null mixer definition has the form:  空的混口口那个器定义格式
	 *
	 *   Z:
	 *
	 * Simple Mixer  简单的混控器
	 * ............
	 *
	 * A simple mixer definition begins with:
	 *
	 *   M: <control count>
	 *   O: <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
	 *
	 * The definition continues with <control count> entries describing the control
	 * inputs and their scaling, in the form:
	 *
	 *   S: <group> <index> <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
	 *
	 * Multirotor Mixer
	 * ................
	 *
	 * The multirotor mixer definition is a single line of the form:
	 *
	 * R: <geometry> <roll scale> <pitch scale> <yaw scale> <deadband>
	 *
	 * @param buf			The mixer configuration buffer.  混控器配置缓冲区
	 * @param buflen		The length of the buffer, updated to reflect  缓冲区长度
	 *				bytes as they are consumed.
	 * @return			Zero on successful load, nonzero otherwise.  返回0代表成功
	 */
	int				load_from_buf(const char *buf, unsigned &buflen);  //从缓冲区中载入

private:
	Mixer				*_first;	/**< linked list of mixers */

	/* do not allow to copy due to pointer data members */
	MixerGroup(const MixerGroup &);
	MixerGroup operator=(const MixerGroup &);
};

/**
 * Null mixer; returns zero.
 *
 * Used as a placeholder for output channels that are unassigned in groups.  在组中未赋值输出通道作为一个占位符
 */
class __EXPORT NullMixer : public Mixer
{
public:
	NullMixer();
	~NullMixer() {};

	/**
	 * Factory method.  工厂方法
	 *
	 * Given a pointer to a buffer containing a text description of the mixer,
	 * returns a pointer to a new instance of the mixer.
	 *
	 * @param buf			Buffer containing a text description of
	 *				the mixer.
	 * @param buflen		Length of the buffer in bytes, adjusted
	 *				to reflect the bytes consumed.
	 * @return			A new NullMixer instance, or nullptr
	 *				if the text format is bad.
	 */
	static NullMixer		*from_text(const char *buf, unsigned &buflen);

	virtual unsigned		mix(float *outputs, unsigned space, uint16_t *status_reg);
	virtual void			groups_required(uint32_t &groups);
};

/**
 * Simple summing mixer.  简单加法混控器
 *
 * Collects zero or more inputs and mixes them to a single output. 获取0个或者更多的输入，混合他们并作为单输出
 */
class __EXPORT SimpleMixer : public Mixer
{
public:
	/**
	 * Constructor
	 *
	 * @param mixinfo		Mixer configuration.  The pointer passed
	 *				becomes the property of the mixer and
	 *				will be freed when the mixer is deleted.
	 */
	SimpleMixer(ControlCallback control_cb,
		    uintptr_t cb_handle,
		    mixer_simple_s *mixinfo);
	~SimpleMixer();

	/**
	 * Factory method with full external configuration.  完全扩展配置的工厂方法
	 *
	 * Given a pointer to a buffer containing a text description of the mixer,
	 * returns a pointer to a new instance of the mixer.
	 *
	 * @param control_cb		The callback to invoke when fetching a  回调函数，调用获取一个控制值
	 *				control value.
	 * @param cb_handle		Handle passed to the control callback.
	 * @param buf			Buffer containing a text description of
	 *				the mixer.
	 * @param buflen		Length of the buffer in bytes, adjusted
	 *				to reflect the bytes consumed.
	 * @return			A new SimpleMixer instance, or nullptr
	 *				if the text format is bad.
	 */
	static SimpleMixer		*from_text(Mixer::ControlCallback control_cb,
			uintptr_t cb_handle,
			const char *buf,
			unsigned &buflen);

	/**
	 * Factory method for PWM/PPM input to internal float representation.  ppm/pwm输入内部浮点表示，工厂模式
	 *
	 * @param control_cb		The callback to invoke when fetching a
	 *				control value.
	 * @param cb_handle		Handle passed to the control callback.
	 * @param input			The control index used when fetching the input.
	 * @param min			The PWM/PPM value considered to be "minimum" (gives -1.0 out)
	 * @param mid			The PWM/PPM value considered to be the midpoint (gives 0.0 out)
	 * @param max			The PWM/PPM value considered to be "maximum" (gives 1.0 out)
	 * @return			A new SimpleMixer instance, or nullptr if one could not be
	 *				allocated.
	 */
	static SimpleMixer		*pwm_input(Mixer::ControlCallback control_cb,
			uintptr_t cb_handle,
			unsigned input,
			uint16_t min,
			uint16_t mid,
			uint16_t max);

	virtual unsigned		mix(float *outputs, unsigned space, uint16_t *status_reg);
	virtual void			groups_required(uint32_t &groups);

	/**
	 * Check that the mixer configuration as loaded is sensible.
	 *
	 * Note that this function will call control_cb, but only cares about
	 * error returns, not the input value.
	 *
	 * @return			Zero if the mixer makes sense, nonzero otherwise.
	 */
	int				check();

protected:

private:
	mixer_simple_s			*_info;

	static int			parse_output_scaler(const char *buf, unsigned &buflen, mixer_scaler_s &scaler);  //解析输出缩放
	static int			parse_control_scaler(const char *buf,
			unsigned &buflen,
			mixer_scaler_s &scaler,
			uint8_t &control_group,
			uint8_t &control_index); //解析控制缩放

	/* do not allow to copy due to ptr data members */
	SimpleMixer(const SimpleMixer &);
	SimpleMixer operator=(const SimpleMixer &);
};

/**
 * Supported multirotor geometries. 支持多旋翼几个形状
 *
 * Values are generated by the multi_tables script and placed to mixer_multirotor.generated.h
 */
typedef unsigned int MultirotorGeometryUnderlyingType;
enum class MultirotorGeometry : MultirotorGeometryUnderlyingType;  //枚举类？

	/**
	 * Multi-rotor mixer for pre-defined vehicle geometries.  域定义的飞行器几何结构的混控器
	 *
	 * Collects four inputs (roll, pitch, yaw, thrust) and mixes them to  获取4个输入（roll，pitch，yaw，thrust推力/油门）并混合他们基于定义的几何结构，作为一系列的输出
	 * a set of outputs based on the configured geometry.
	 */
	class __EXPORT MultirotorMixer : public Mixer
{
public:
	/**

	 * Precalculated rotor mix. 预先计算旋转混合
	 */
	struct Rotor {
		float	roll_scale;	/**< scales roll for this rotor */  //roll轴旋转的缩放因子
		float	pitch_scale;	/**< scales pitch for this rotor */
		float	yaw_scale;	/**< scales yaw for this rotor */
		float	out_scale;	/**< scales total out for this rotor */
	};

	/**
	 * Constructor.
	 *
	 * @param control_cb		Callback invoked to read inputs.  回调函数，读取输入
	 * @param cb_handle		Passed to control_cb.
	 * @param geometry		The selected geometry.  选择几何结构
	 * @param roll_scale		Scaling factor applied to roll inputs
	 *				compared to thrust.  应用于roll轴的输入缩放因子，对比与推力/油门
	 * @param pitch_scale		Scaling factor applied to pitch inputs
	 *				compared to thrust.
	 * @param yaw_wcale		Scaling factor applied to yaw inputs compared
	 *				to thrust.
	 * @param idle_speed		Minimum rotor control output value; usually
	 *				tuned to ensure that rotors never stall at the
	 * 				low end of their control range.
	 */
	MultirotorMixer(ControlCallback control_cb,
			uintptr_t cb_handle,
			MultirotorGeometry geometry,
			float roll_scale,
			float pitch_scale,
			float yaw_scale,
			float idle_speed);
	~MultirotorMixer();

	/**
	 * Factory method.  工厂模式
	 *
	 * Given a pointer to a buffer containing a text description of the mixer,
	 * returns a pointer to a new instance of the mixer.
	 *
	 * @param control_cb		The callback to invoke when fetching a  回调函数，调用后获取控制值
	 *				control value.
	 * @param cb_handle		Handle passed to the control callback.
	 * @param buf			Buffer containing a text description of
	 *				the mixer.
	 * @param buflen		Length of the buffer in bytes, adjusted
	 *				to reflect the bytes consumed.
	 * @return			A new MultirotorMixer instance, or nullptr
	 *				if the text format is bad.
	 */
	static MultirotorMixer		*from_text(Mixer::ControlCallback control_cb,
			uintptr_t cb_handle,
			const char *buf,
			unsigned &buflen);

	virtual unsigned		mix(float *outputs, unsigned space, uint16_t *status_reg);
	virtual void			groups_required(uint32_t &groups);

private:
	float				_roll_scale;
	float				_pitch_scale;
	float				_yaw_scale;
	float				_idle_speed;

	orb_advert_t			_limits_pub;
	multirotor_motor_limits_s 	_limits;

	unsigned			_rotor_count;
	const Rotor			*_rotors;

	/* do not allow to copy due to ptr data members */
	MultirotorMixer(const MultirotorMixer &);
	MultirotorMixer operator=(const MultirotorMixer &);
};

#endif
