#ifndef _uORBCommon_hpp_
#define _uORBCommon_hpp_

#include <drivers/device/device_nuttx.h>
#include <drivers/drv_orb_dev.h>
#include <systemlib/err.h>
#include "uORB.h"
#include <drivers/drv_hrt.h>


namespace uORB
{
static const unsigned orb_maxpath = 64;

#ifdef ERROR
# undef ERROR
#endif
/* ERROR is not defined for c++ */
const int ERROR = -1;

enum Flavor {
    PUBSUB,  //发布订阅
    PARAM    //参数
};

struct orb_advertdata {
	const struct orb_metadata *meta;
	int *instance;
	int priority;
};
}
#endif // _uORBCommon_hpp_
