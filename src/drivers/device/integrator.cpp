/**
 * @file integrator.cpp
 *
 * A resettable integrator 一个可复位的积分器
 *
 */

#include "integrator.h"

Integrator::Integrator(uint64_t auto_reset_interval, bool coning_compensation) :
	_auto_reset_interval(auto_reset_interval),
	_last_integration(0),
	_last_auto(0),
	_integral_auto(0.0f, 0.0f, 0.0f),
	_integral_read(0.0f, 0.0f, 0.0f),
	_last_val(0.0f, 0.0f, 0.0f),
	_last_delta(0.0f, 0.0f, 0.0f),
	_auto_callback(nullptr),
	_coning_comp_on(coning_compensation)
{

}

Integrator::~Integrator()
{

}

//积分器中输入一个项目
bool
Integrator::put(uint64_t timestamp, math::Vector<3> &val, math::Vector<3> &integral, uint64_t &integral_dt)
{
	bool auto_reset = false;  //自动复位标志

	if (_last_integration == 0) {  //如果最后一次积分数据为0
		/* this is the first item in the integrator */  //积分器中第一个项目
		_last_integration = timestamp;
		_last_auto = timestamp;
		_last_val = val;  //存入积分器的项目
		return false;
	}

	// Integrate  积分
	double dt = (double)(timestamp - _last_integration) / 1000000.0;  //时间
	math::Vector<3> i = (val + _last_val) * dt * 0.5f; //当前值加之前值

	// Apply coning compensation if required
	if (_coning_comp_on) {  //锥形补偿
		// Coning compensation derived by Paul Riseborough and Jonathan Challinger,
		// following:
		// Tian et al (2010) Three-loop Integration of GPS and Strapdown INS with Coning and Sculling Compensation
		// Available: http://www.sage.unsw.edu.au/snap/publications/tian_etal2010b.pdf

		i += ((_integral_auto + _last_delta * (1.0f / 6.0f)) % i) * 0.5f;
	}

	_integral_auto += i;
	_integral_read += i;

	_last_integration = timestamp;
	_last_val = val;
	_last_delta = i;

	if ((timestamp - _last_auto) > _auto_reset_interval) {  //大于自动复位间隔
		if (_auto_callback) {
			/* call the callback */
			_auto_callback(timestamp, _integral_auto);
		}

		integral = _integral_auto;
		integral_dt = (timestamp - _last_auto);

		auto_reset = true;
		_last_auto = timestamp;
		_integral_auto(0) = 0.0f;
		_integral_auto(1) = 0.0f;
		_integral_auto(2) = 0.0f;
	}

	return auto_reset;
}

//从积分器中读取
math::Vector<3>
Integrator::read(bool auto_reset)
{
	math::Vector<3> val = _integral_read;  //最后一次数据存入变量，并返回

	if (auto_reset) { //如果自动复位标志置位，清楚数据
		_integral_read(0) = 0.0f;
		_integral_read(1) = 0.0f;
		_integral_read(2) = 0.0f;
	}

	return val;
}






