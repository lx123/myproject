
/**
 * @file drv_accel.h
 *
 * Accelerometer driver interface.
 */

#ifndef _DRV_ACCEL_H
#define _DRV_ACCEL_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define ACCEL_BASE_DEVICE_PATH	"/dev/accel"
#define ACCEL0_DEVICE_PATH	"/dev/accel0"
#define ACCEL1_DEVICE_PATH	"/dev/accel1"
#define ACCEL2_DEVICE_PATH	"/dev/accel2"

#include <uORB/topics/sensor_accel.h>
#define accel_report sensor_accel_s

/** accel scaling factors; Vout = Vscale * (Vin + Voffset) */
struct accel_scale {
	float	x_offset;
	float	x_scale;
	float	y_offset;
	float	y_scale;
	float	z_offset;
	float	z_scale;
};

/*
 * ioctl() definitions
 *
 * Accelerometer drivers also implement the generic sensor driver
 * interfaces from drv_sensor.h
 */

#define _ACCELIOCBASE		(0x2100)
#define _ACCELIOC(_n)		(_PX4_IOC(_ACCELIOCBASE, _n))


/** set the accel internal sample rate to at least (arg) Hz */
#define ACCELIOCSSAMPLERATE	_ACCELIOC(0)

#define ACCEL_SAMPLERATE_DEFAULT    1000003	/**< default sample rate */

/** return the accel internal sample rate in Hz */
#define ACCELIOCGSAMPLERATE	_ACCELIOC(1)

/** set the accel internal lowpass filter to no lower than (arg) Hz */
#define ACCELIOCSLOWPASS	_ACCELIOC(2)

/** return the accel internal lowpass filter in Hz */
#define ACCELIOCGLOWPASS	_ACCELIOC(3)

/** set the accel scaling constants to the structure pointed to by (arg) */
#define ACCELIOCSSCALE		_ACCELIOC(5)

/** get the accel scaling constants into the structure pointed to by (arg) */
#define ACCELIOCGSCALE		_ACCELIOC(6)

/** set the accel measurement range to handle at least (arg) g */
#define ACCELIOCSRANGE		_ACCELIOC(7)

/** get the current accel measurement range in g */
#define ACCELIOCGRANGE		_ACCELIOC(8)

/** get the result of a sensor self-test */
#define ACCELIOCSELFTEST	_ACCELIOC(9)

#endif /* _DRV_ACCEL_H */






