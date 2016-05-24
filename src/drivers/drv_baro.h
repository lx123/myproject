/**
 * @file drv_baro.h
 *
 * Barometric pressure sensor driver interface.
 */

#ifndef _DRV_BARO_H
#define _DRV_BARO_H

#include <px4_defines.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define BARO_BASE_DEVICE_PATH	"/dev/baro"
#define BARO0_DEVICE_PATH	"/dev/baro0"

#include <uORB/topics/sensor_baro.h>
#define baro_report sensor_baro_s

/*
 * ioctl() definitions
 */

#define _BAROIOCBASE		(0x2200)
#define _BAROIOC(_n)		(_PX4_IOC(_BAROIOCBASE, _n))

/** set corrected MSL pressure in pascals */
#define BAROIOCSMSLPRESSURE	_BAROIOC(0)

/** get current MSL pressure in pascals */
#define BAROIOCGMSLPRESSURE	_BAROIOC(1)

#endif /* _DRV_BARO_H */
