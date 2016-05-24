/**
 * @file integrator.h
 *
 * A resettable integrator
 *
 */


#include <mathlib/mathlib.h>
//单独的类
class Integrator
{
public:
	Integrator(uint64_t auto_reset_interval = 4000 /* 250 Hz */, bool coning_compensation = false); //默认自动复位间隔250hz，时间间隔单位us
	virtual ~Integrator();

	/**
	 * Put an item into the integral.
	 *
	 * @param timestamp	Timestamp of the current value
	 * @param val		Item to put
	 * @param integral	Current integral in case the integrator did reset, else the value will not be modified
	 * @return		true if putting the item triggered an integral reset
	 *			and the integral should be published
	 */
	bool			put(uint64_t timestamp, math::Vector<3> &val, math::Vector<3> &integral, uint64_t &integral_dt);  //加入一个项目到积分器

	/**
	 * Get the current integral value
	 *
	 * @return		the integral since the last auto-reset
	 */
	math::Vector<3>		get() { return _integral_auto; }  //获取当前积分器的值

	/**
	 * Read from the integral
	 *
	 * @param auto_reset	Reset the integral to zero on read
	 * @return		the integral since the last read-reset
	 */
	math::Vector<3>		read(bool auto_reset);  //从积分器中读取

	/**
	 * Get current integral start time
	 */
	uint64_t		current_integral_start() { return _last_auto; }  //获取当前积分器开始时间

private:
	uint64_t _auto_reset_interval;		/**< the interval after which the content will be published and the integrator reset */  //自动复位间隔
	uint64_t _last_integration;			/**< timestamp of the last integration step */  //上次积分间隔时间戳
	uint64_t _last_auto;				/**< last auto-announcement of integral value */  //最后一次自动公告积分器的数值
	math::Vector<3> _integral_auto;			/**< the integrated value which auto-resets after _auto_reset_interval */ //自动复位
	math::Vector<3> _integral_read;			/**< the integrated value since the last read */ //最后一次读取的积分器数值
	math::Vector<3> _last_val;			/**< previously integrated last value */  //之前积分器最后的值
	math::Vector<3> _last_delta;			/**< last local delta */
	void (*_auto_callback)(uint64_t, math::Vector<3>);	/**< the function callback for auto-reset */  //callback自动复位
	bool _coning_comp_on;				/**< coning compensation */  //锥形补偿

	/* we don't want this class to be copied */
	Integrator(const Integrator &);
	Integrator operator=(const Integrator &);
};





