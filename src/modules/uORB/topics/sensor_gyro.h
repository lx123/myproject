
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
struct __EXPORT sensor_gyro_s {
#else
struct sensor_gyro_s {
#endif
	uint64_t timestamp;
	uint64_t integral_dt;
	uint64_t error_count;
	float x;
	float y;
	float z;
	float x_integral;
	float y_integral;
	float z_integral;
	float temperature;
	float range_rad_s;
	float scaling;
	int16_t x_raw;
	int16_t y_raw;
	int16_t z_raw;
	int16_t temperature_raw;
	uint32_t device_id;
#ifdef __cplusplus

#endif
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(sensor_gyro);
