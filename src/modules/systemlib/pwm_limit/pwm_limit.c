
/**
 * @file pwm_limit.c
 *
 * Library for PWM output limiting
 *
 * @author Julian Oes <julian@px4.io>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "pwm_limit.h"
#include <math.h>
#include <stdbool.h>
#include <drivers/drv_hrt.h>
#include <stdio.h>

#define PROGRESS_INT_SCALING	10000

void pwm_limit_init(pwm_limit_t *limit)
{
	limit->state = PWM_LIMIT_STATE_INIT;  //pwm限制初始化状态
	limit->time_armed = 0;  //时间解锁=0
	return;
}

void pwm_limit_calc(const bool armed, const bool pre_armed, const unsigned num_channels, const uint16_t reverse_mask,
		    const uint16_t *disarmed_pwm, const uint16_t *min_pwm, const uint16_t *max_pwm,
		    const float *output, uint16_t *effective_pwm, pwm_limit_t *limit)
{

	/* first evaluate state changes */
	switch (limit->state) {
	case PWM_LIMIT_STATE_INIT:

		if (armed) {

			/* set arming time for the first call */
			if (limit->time_armed == 0) {
				limit->time_armed = hrt_absolute_time();
			}

			if (hrt_elapsed_time(&limit->time_armed) >= INIT_TIME_US) {
				limit->state = PWM_LIMIT_STATE_OFF;
			}
		}

		break;

	case PWM_LIMIT_STATE_OFF:
		if (armed) {
			limit->state = PWM_LIMIT_STATE_RAMP;

			/* reset arming time, used for ramp timing */
			limit->time_armed = hrt_absolute_time();
		}

		break;

	case PWM_LIMIT_STATE_RAMP:
		if (!armed) {
			limit->state = PWM_LIMIT_STATE_OFF;

		} else if (hrt_elapsed_time(&limit->time_armed) >= RAMP_TIME_US) {
			limit->state = PWM_LIMIT_STATE_ON;
		}

		break;

	case PWM_LIMIT_STATE_ON:
		if (!armed) {
			limit->state = PWM_LIMIT_STATE_OFF;
		}

		break;

	default:
		break;
	}

	/* if the system is pre-armed, the limit state is temporarily on,
	 * as some outputs are valid and the non-valid outputs have been
	 * set to NaN. This is not stored in the state machine though,
	 * as the throttle channels need to go through the ramp at
	 * regular arming time.
	 */

	unsigned local_limit_state = limit->state;

	if (pre_armed) {
		local_limit_state = PWM_LIMIT_STATE_ON;
	}

	unsigned progress;

	/* then set effective_pwm based on state */
	switch (local_limit_state) {
	case PWM_LIMIT_STATE_OFF:
	case PWM_LIMIT_STATE_INIT:
		for (unsigned i = 0; i < num_channels; i++) {
			effective_pwm[i] = disarmed_pwm[i];
		}

		break;

	case PWM_LIMIT_STATE_RAMP: {
			hrt_abstime diff = hrt_elapsed_time(&limit->time_armed);

			progress = diff * PROGRESS_INT_SCALING / RAMP_TIME_US;

			if (progress > PROGRESS_INT_SCALING) {
				progress = PROGRESS_INT_SCALING;
			}

			for (unsigned i = 0; i < num_channels; i++) {

				float control_value = output[i];

				/* check for invalid / disabled channels */
				if (!isfinite(control_value)) {
					effective_pwm[i] = disarmed_pwm[i];
					continue;
				}

				uint16_t ramp_min_pwm;

				/* if a disarmed pwm value was set, blend between disarmed and min */
				if (disarmed_pwm[i] > 0) {

					/* safeguard against overflows */
					unsigned disarmed = disarmed_pwm[i];

					if (disarmed > min_pwm[i]) {
						disarmed = min_pwm[i];
					}

					unsigned disarmed_min_diff = min_pwm[i] - disarmed;
					ramp_min_pwm = disarmed + (disarmed_min_diff * progress) / PROGRESS_INT_SCALING;

				} else {

					/* no disarmed pwm value set, choose min pwm */
					ramp_min_pwm = min_pwm[i];
				}

				if (reverse_mask & (1 << i)) {
					control_value = -1.0f * control_value;
				}

				effective_pwm[i] = control_value * (max_pwm[i] - ramp_min_pwm) / 2 + (max_pwm[i] + ramp_min_pwm) / 2;

				/* last line of defense against invalid inputs */
				if (effective_pwm[i] < ramp_min_pwm) {
					effective_pwm[i] = ramp_min_pwm;

				} else if (effective_pwm[i] > max_pwm[i]) {
					effective_pwm[i] = max_pwm[i];
				}
			}
		}
		break;

	case PWM_LIMIT_STATE_ON:
		for (unsigned i = 0; i < num_channels; i++) {

			float control_value = output[i];

			/* check for invalid / disabled channels */
			if (!isfinite(control_value)) {
				effective_pwm[i] = disarmed_pwm[i];
				continue;
			}

			if (reverse_mask & (1 << i)) {
				control_value = -1.0f * control_value;
			}

			effective_pwm[i] = control_value * (max_pwm[i] - min_pwm[i]) / 2 + (max_pwm[i] + min_pwm[i]) / 2;

			/* last line of defense against invalid inputs */
			if (effective_pwm[i] < min_pwm[i]) {
				effective_pwm[i] = min_pwm[i];

			} else if (effective_pwm[i] > max_pwm[i]) {
				effective_pwm[i] = max_pwm[i];
			}
		}

		break;

	default:
		break;
	}

	return;
}
