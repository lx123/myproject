
/**
 * @file Publication.cpp
 *
 */

#include "Publication.hpp"
//#include "topics/vehicle_attitude.h"
//#include "topics/vehicle_local_position.h"
//#include "topics/vehicle_global_position.h"
//#include "topics/debug_key_value.h"
//#include "topics/actuator_controls.h"
//#include "topics/vehicle_global_velocity_setpoint.h"
//#include "topics/vehicle_attitude_setpoint.h"
//#include "topics/vehicle_rates_setpoint.h"
//#include "topics/actuator_outputs.h"
//#include "topics/actuator_direct.h"
//#include "topics/encoders.h"
//#include "topics/tecs_status.h"
//#include "topics/rc_channels.h"
//#include "topics/filtered_bottom_flow.h"

#include <px4_defines.h>

namespace uORB
{
//发布者基类构造函数
PublicationBase::PublicationBase(const struct orb_metadata *meta,
				 int priority) :
	_meta(meta),
	_priority(priority),
	_instance(),
	_handle(nullptr)
{
}
//发布者基类更新
void PublicationBase::update(void *data)
{
	if (_handle != nullptr) {
		int ret = orb_publish(getMeta(), getHandle(), data);  //发布

		if (ret != PX4_OK) { warnx("publish fail"); }  //发布失败

	} else {
		orb_advert_t handle;

		if (_priority > 0) {
			handle = orb_advertise_multi(
					 getMeta(), data,
					 &_instance, _priority);

		} else {
			handle = orb_advertise(getMeta(), data);
		}

		if (int64_t(handle) != PX4_ERROR) {
			setHandle(handle);

		} else {
			warnx("advert fail");
		}
	}
}

PublicationBase::~PublicationBase()
{
}

PublicationNode::PublicationNode(const struct orb_metadata *meta,
				 int priority,
				 List<PublicationNode *> *list) :
	PublicationBase(meta, priority)
{
	if (list != nullptr) { list->add(this); }
}

// explicit template instantiation
//template class __EXPORT Publication<vehicle_attitude_s>;
//template class __EXPORT Publication<vehicle_local_position_s>;
//template class __EXPORT Publication<vehicle_global_position_s>;
//template class __EXPORT Publication<debug_key_value_s>;
//template class __EXPORT Publication<actuator_controls_s>;
//template class __EXPORT Publication<vehicle_global_velocity_setpoint_s>;
//template class __EXPORT Publication<vehicle_attitude_setpoint_s>;
//template class __EXPORT Publication<vehicle_rates_setpoint_s>;
//template class __EXPORT Publication<actuator_outputs_s>;
//template class __EXPORT Publication<actuator_direct_s>;
//template class __EXPORT Publication<encoders_s>;
//template class __EXPORT Publication<tecs_status_s>;
//template class __EXPORT Publication<rc_channels_s>;
//template class __EXPORT Publication<filtered_bottom_flow_s>;

}








