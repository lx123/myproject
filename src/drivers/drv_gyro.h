/**
 * @file drv_gyro.h
 *
 * Gyroscope driver interface.
 */

#ifndef _DRV_GYRO_H
#define _DRV_GYRO_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define GYRO_BASE_DEVICE_PATH	"/dev/gyro"
#define GYRO0_DEVICE_PATH	"/dev/gyro0"
#define GYRO1_DEVICE_PATH	"/dev/gyro1"
#define GYRO2_DEVICE_PATH	"/dev/gyro2"

#include <uORB/topics/sensor_gyro.h>
#define gyro_report sensor_gyro_s

/** gyro scaling factors; Vout = (Vin * Vscale) + Voffset */
struct gyro_scale {
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

#define _GYROIOCBASE		(0x2300)
#define _GYROIOC(_n)		(_PX4_IOC(_GYROIOCBASE, _n))

/** set the gyro internal sample rate to at least (arg) Hz */
#define GYROIOCSSAMPLERATE	_GYROIOC(0)

#define GYRO_SAMPLERATE_DEFAULT    1000003	/**< default sample rate */  //默认的采样速率

/** return the gyro internal sample rate in Hz */
#define GYROIOCGSAMPLERATE	_GYROIOC(1)

/** set the gyro internal lowpass filter to no lower than (arg) Hz */
#define GYROIOCSLOWPASS		_GYROIOC(2)

/** set the gyro internal lowpass filter to no lower than (arg) Hz */
#define GYROIOCGLOWPASS		_GYROIOC(3)

/** set the gyro scaling constants to (arg) */
#define GYROIOCSSCALE		_GYROIOC(4)

/** get the gyro scaling constants into (arg) */
#define GYROIOCGSCALE		_GYROIOC(5)

/** set the gyro measurement range to handle at least (arg) degrees per second */
#define GYROIOCSRANGE		_GYROIOC(6)

/** get the current gyro measurement range in degrees per second */
#define GYROIOCGRANGE		_GYROIOC(7)

/** check the status of the sensor */
#define GYROIOCSELFTEST		_GYROIOC(8)

#endif /* _DRV_GYRO_H */






