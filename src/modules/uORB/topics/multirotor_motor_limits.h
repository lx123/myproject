
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
struct __EXPORT multirotor_motor_limits_s {
#else
struct multirotor_motor_limits_s {
#endif
	uint8_t lower_limit;
	uint8_t upper_limit;
	uint8_t yaw;
	uint8_t reserved;
#ifdef __cplusplus

#endif
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(multirotor_motor_limits);

