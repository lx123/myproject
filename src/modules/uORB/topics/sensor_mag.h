
#pragma once

#include <stdint.h>
#include <uORB/uORB.h>


#ifndef __cplusplus

#endif

/**
 * @addtogroup topics
 * @{
 */


#ifdef __cplusplus
struct __EXPORT sensor_mag_s {
#else
struct sensor_mag_s {
#endif
	uint64_t timestamp;
	uint64_t error_count;
	float x;
	float y;
	float z;
	float range_ga;
	float scaling;
	float temperature;
	int16_t x_raw;
	int16_t y_raw;
	int16_t z_raw;
#ifdef __cplusplus

#endif
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(sensor_mag);


