
/**
 * @file Subscription.cpp
 *
 */

#include "Subscription.hpp"
//#include "topics/parameter_update.h"
//#include "topics/actuator_controls.h"
//#include "topics/vehicle_gps_position.h"
//#include "topics/satellite_info.h"
//#include "topics/sensor_combined.h"
//#include "topics/hil_sensor.h"
//#include "topics/vehicle_attitude.h"
//#include "topics/vehicle_global_position.h"
//#include "topics/encoders.h"
//#include "topics/position_setpoint_triplet.h"
//#include "topics/vehicle_status.h"
//#include "topics/manual_control_setpoint.h"
//#include "topics/vehicle_local_position_setpoint.h"
//#include "topics/vehicle_local_position.h"
//#include "topics/vehicle_attitude_setpoint.h"
//#include "topics/vehicle_rates_setpoint.h"
//#include "topics/rc_channels.h"
//#include "topics/battery_status.h"
//#include "topics/optical_flow.h"
//#include "topics/distance_sensor.h"
//#include "topics/home_position.h"
//#include "topics/vehicle_control_mode.h"
//#include "topics/actuator_armed.h"
//#include "topics/att_pos_mocap.h"
//#include "topics/vision_position_estimate.h"

#include <px4_defines.h>

namespace uORB
{

SubscriptionBase::SubscriptionBase(const struct orb_metadata *meta,  //订阅者基类构造函数
				   unsigned interval, unsigned instance) :
	_meta(meta),  //元数据结构
	_instance(instance),  //多个会话标志
	_handle()   //句柄
{
	if (_instance > 0) { //订阅多个主题
		_handle =  orb_subscribe_multi(
				   getMeta(), instance);

	} else {  //订阅一个主题
		_handle =  orb_subscribe(getMeta());
	}

	if (_handle < 0) { warnx("sub failed"); }  //订阅失败

	if (interval > 0) {  //如果时间间隔大于0
		orb_set_interval(getHandle(), interval);  //设置会话时间间隔
	}
}

bool SubscriptionBase::updated()  //更新
{
	bool isUpdated = false;  //更新标志位
	int ret = orb_check(_handle, &isUpdated); //检查

	if (ret != PX4_OK) { warnx("orb check failed"); }  //会话检查失败

	return isUpdated;  //返回检查标志
}

void SubscriptionBase::update(void *data)  //更新
{
	if (updated()) {  //如果有更新
		int ret = orb_copy(_meta, _handle, data);  //取出数据

		if (ret != PX4_OK) { warnx("orb copy failed"); }  //复制失败
	}
}

SubscriptionBase::~SubscriptionBase()  //基类析构
{
	int ret = orb_unsubscribe(_handle);  //取消订阅主题

	if (ret != PX4_OK) { warnx("orb unsubscribe failed"); }  //取消失败
}

template <class T>  //模板
Subscription<T>::Subscription(const struct orb_metadata *meta,  //构造函数
			      unsigned interval,
			      int instance,
			      List<SubscriptionNode *> *list) :
	SubscriptionNode(meta, interval, instance, list),   //初始化订阅者节点
	_data() // initialize data structure to zero
{
}

template <class T>
Subscription<T>::~Subscription()  //析构函数
{
}

template <class T>
void Subscription<T>::update()
{
	SubscriptionBase::update((void *)(&_data));
}

//template <class T>
//const T &Subscription<T>::get() { return _data; }

//template class __EXPORT Subscription<parameter_update_s>;
//template class __EXPORT Subscription<actuator_controls_s>;
//template class __EXPORT Subscription<vehicle_gps_position_s>;
//template class __EXPORT Subscription<satellite_info_s>;
//template class __EXPORT Subscription<sensor_combined_s>;
//template class __EXPORT Subscription<hil_sensor_s>;
//template class __EXPORT Subscription<vehicle_attitude_s>;
//template class __EXPORT Subscription<vehicle_global_position_s>;
//template class __EXPORT Subscription<encoders_s>;
//template class __EXPORT Subscription<position_setpoint_triplet_s>;
//template class __EXPORT Subscription<vehicle_status_s>;
//template class __EXPORT Subscription<manual_control_setpoint_s>;
//template class __EXPORT Subscription<vehicle_local_position_setpoint_s>;
//template class __EXPORT Subscription<vehicle_local_position_s>;
//template class __EXPORT Subscription<vehicle_attitude_setpoint_s>;
//template class __EXPORT Subscription<vehicle_rates_setpoint_s>;
//template class __EXPORT Subscription<rc_channels_s>;
//template class __EXPORT Subscription<vehicle_control_mode_s>;
//template class __EXPORT Subscription<actuator_armed_s>;
//template class __EXPORT Subscription<battery_status_s>;
//template class __EXPORT Subscription<home_position_s>;
//template class __EXPORT Subscription<optical_flow_s>;
//template class __EXPORT Subscription<distance_sensor_s>;
//template class __EXPORT Subscription<att_pos_mocap_s>;
//template class __EXPORT Subscription<vision_position_estimate_s>;

} // namespace uORB



