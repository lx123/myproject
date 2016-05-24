
/**
 * @file Magnetometer driver interface.
 */

#ifndef _DRV_MAG_H
#define _DRV_MAG_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define MAG_BASE_DEVICE_PATH	"/dev/mag"
#define MAG0_DEVICE_PATH	"/dev/mag0"
#define MAG1_DEVICE_PATH	"/dev/mag1"
#define MAG2_DEVICE_PATH	"/dev/mag2"

#include <uORB/topics/sensor_mag.h>
#define mag_report sensor_mag_s

/** mag scaling factors; Vout = (Vin * Vscale) + Voffset */
struct mag_scale {
	float	x_offset;
	float	x_scale;
	float	y_offset;
	float	y_scale;
	float	z_offset;
	float	z_scale;
};

/*
 * ioctl() definitions
 */

#define _MAGIOCBASE		(0x2400)
#define _MAGIOC(_n)		(_PX4_IOC(_MAGIOCBASE, _n))

/** set the mag internal sample rate to at least (arg) Hz */
#define MAGIOCSSAMPLERATE	_MAGIOC(0)

/** return the mag internal sample rate in Hz */
#define MAGIOCGSAMPLERATE	_MAGIOC(1)

/** set the mag internal lowpass filter to no lower than (arg) Hz */
#define MAGIOCSLOWPASS		_MAGIOC(2)

/** return the mag internal lowpass filter in Hz */
#define MAGIOCGLOWPASS		_MAGIOC(3)

/** set the mag scaling constants to the structure pointed to by (arg) */
#define MAGIOCSSCALE		_MAGIOC(4)

/** copy the mag scaling constants to the structure pointed to by (arg) */
#define MAGIOCGSCALE		_MAGIOC(5)

/** set the measurement range to handle (at least) arg Gauss */
#define MAGIOCSRANGE		_MAGIOC(6)

/** return the current mag measurement range in Gauss */
#define MAGIOCGRANGE		_MAGIOC(7)

/** perform self-calibration, update scale factors to canonical units */
#define MAGIOCCALIBRATE		_MAGIOC(8)

/** excite strap */
#define MAGIOCEXSTRAP		_MAGIOC(9)

/** perform self test and report status */
#define MAGIOCSELFTEST		_MAGIOC(10)

/** determine if mag is external or onboard */
#define MAGIOCGEXTERNAL		_MAGIOC(11)

/** enable/disable temperature compensation */
#define MAGIOCSTEMPCOMP		_MAGIOC(12)

#endif /* _DRV_MAG_H */


