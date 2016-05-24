
/**
 * @file pwm_limit.c
 *
 * Library for PWM output limiting  pwm输出限制库
 *
 * @author Julian Oes <julian@px4.io>
 */

#ifndef PWM_LIMIT_H_
#define PWM_LIMIT_H_

#include <stdint.h>
#include <stdbool.h>

__BEGIN_DECLS

/*
 * time for the ESCs to initialize
 * (this is not actually needed if PWM is sent right after boot)
 */
#define INIT_TIME_US 500000   //电调初始化时间
/*
 * time to slowly ramp up the ESCs
 */
#define RAMP_TIME_US 2500000   //缓慢提升电调速度

enum pwm_limit_state {
	PWM_LIMIT_STATE_OFF = 0,
	PWM_LIMIT_STATE_INIT,   //电调初始化
	PWM_LIMIT_STATE_RAMP,  //电调速度斜坡上升
	PWM_LIMIT_STATE_ON
};

typedef struct {
	enum pwm_limit_state state;   //pwm限制状态
	uint64_t time_armed;
} pwm_limit_t;

__EXPORT void pwm_limit_init(pwm_limit_t *limit);  //pwm限制初始化

__EXPORT void pwm_limit_calc(const bool armed, const bool pre_armed, const unsigned num_channels,   //pwm限制计算
			     const uint16_t reverse_mask, const uint16_t *disarmed_pwm,
			     const uint16_t *min_pwm, const uint16_t *max_pwm, const float *output, uint16_t *effective_pwm, pwm_limit_t *limit);

__END_DECLS

#endif /* PWM_LIMIT_H_ */
