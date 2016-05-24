
/**
 * @file drv_device.h
 *
 * Generic device / sensor interface.   通用设备/传感器接口
 */

#ifndef _DRV_DEVICE_H
#define _DRV_DEVICE_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#include "DevIOCTL.h"

#ifdef __PX4_POSIX

#ifndef SIOCDEVPRIVATE
#define SIOCDEVPRIVATE 1
#endif

#define DIOC_GETPRIV    SIOCDEVPRIVATE
#endif

#endif /* _DRV_DEVICE_H */

