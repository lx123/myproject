
#ifndef _DRV_UORB_H
#define _DRV_UORB_H

/**
 * @file drv_orb_dev.h
 *
 * uORB published object driver.
 */

#include <px4_defines.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdint.h>

/* XXX for ORB_DECLARE used in many drivers */
#include "../modules/uORB/uORB.h"

/*
 * ioctl() definitions
 */

/** path to the uORB control device for pub/sub topics */  //uORB控制设备 发布/订阅主题
#define TOPIC_MASTER_DEVICE_PATH	"/obj/_obj_"

/** path to the uORB control device for parameter topics */   //参数会话的uORB控制设备路径
#define PARAM_MASTER_DEVICE_PATH	"/param/_param_"

/** maximum ogbject name length */
#define ORB_MAXNAME		32  //最大主题名称长度

#define _ORBIOCBASE		(0x2600)
#define _ORBIOC(_n)		(_PX4_IOC(_ORBIOCBASE, _n))

/*
 * IOCTLs for the uORB control device
 */

/** Advertise a new topic described by *(uorb_metadata *)arg */
#define ORBIOCADVERTISE		_ORBIOC(0)

/*
 * IOCTLs for individual topics.
 */

/** Fetch the time at which the topic was last updated into *(uint64_t *)arg */
#define ORBIOCLASTUPDATE	_ORBIOC(10)

/** Check whether the topic has been updated since it was last read, sets *(bool *)arg */
#define ORBIOCUPDATED		_ORBIOC(11)

/** Set the minimum interval at which the topic can be seen to be updated for this subscription */
#define ORBIOCSETINTERVAL	_ORBIOC(12)

/** Get the global advertiser handle for the topic */
#define ORBIOCGADVERTISER	_ORBIOC(13)

/** Get the priority for the topic */
#define ORBIOCGPRIORITY		_ORBIOC(14)

#endif /* _DRV_UORB_H */
