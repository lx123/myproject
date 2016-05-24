

#pragma once

#include <stdint.h>
#include <uORB/uORB.h>


#ifndef __cplusplus
#define NUM_ACTUATOR_CONTROLS 8
#define NUM_ACTUATOR_CONTROL_GROUPS 4
#define INDEX_ROLL 0
#define INDEX_PITCH 1
#define INDEX_YAW 2
#define INDEX_THROTTLE 3
#define INDEX_FLAPS 4
#define INDEX_SPOILERS 5
#define INDEX_AIRBRAKES 6
#define INDEX_LANDING_GEAR 7
#define GROUP_INDEX_ATTITUDE 0
#define GROUP_INDEX_ATTITUDE_ALTERNATE 1

#endif

/**
 * @addtogroup topics
 * @{
 */


#ifdef __cplusplus
struct __EXPORT actuator_controls_s {
#else
struct actuator_controls_s {
#endif
	uint64_t timestamp;
	uint64_t timestamp_sample;
	float control[8];
#ifdef __cplusplus
	static const uint8_t NUM_ACTUATOR_CONTROLS = 8;
	static const uint8_t NUM_ACTUATOR_CONTROL_GROUPS = 4;
	static const uint8_t INDEX_ROLL = 0;
	static const uint8_t INDEX_PITCH = 1;
	static const uint8_t INDEX_YAW = 2;
	static const uint8_t INDEX_THROTTLE = 3;
	static const uint8_t INDEX_FLAPS = 4;
	static const uint8_t INDEX_SPOILERS = 5;
	static const uint8_t INDEX_AIRBRAKES = 6;
	static const uint8_t INDEX_LANDING_GEAR = 7;
	static const uint8_t GROUP_INDEX_ATTITUDE = 0;
	static const uint8_t GROUP_INDEX_ATTITUDE_ALTERNATE = 1;

#endif
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(actuator_controls);

