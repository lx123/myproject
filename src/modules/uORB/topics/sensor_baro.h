
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
struct __EXPORT sensor_baro_s {
#else
struct sensor_baro_s {
#endif
	float pressure;
	float altitude;
	float temperature;
	uint64_t timestamp;
	uint64_t error_count;
#ifdef __cplusplus

#endif
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(sensor_baro);


