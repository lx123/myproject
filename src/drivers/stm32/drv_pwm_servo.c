
/*
 * @file drv_pwm_servo.c
 *
 * Servo driver supporting PWM servos connected to STM32 timer blocks.
 *
 * Works with any of the 'generic' or 'advanced' STM32 timers that
 * have output pins, does not require an interrupt.
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>

#include "drv_io_timer.h"
#include "drv_pwm_servo.h"

#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>

#include <stm32.h>
#include <stm32_tim.h>

int up_pwm_servo_set(unsigned channel, servo_position_t value)  //pwm servo设置
{
	return io_timer_set_ccr(channel, value);
}

servo_position_t up_pwm_servo_get(unsigned channel)
{
	return io_channel_get_ccr(channel);
}

int up_pwm_servo_init(uint32_t channel_mask)  //pwm servo输出初始化
{
	/* Init channels */
    uint32_t current = io_timer_get_mode_channels(IOTimerChanMode_PWMOut); //通道模式 。值应该是0

    // First free the current set of PWMs    释放当前
//最大通道数量8
	for (unsigned channel = 0; current != 0 &&  channel < MAX_TIMER_IO_CHANNELS; channel++) {  //current=0表示使用了定时器，不进行通道释放
		if (current & (1 << channel)) {  //对定时器掩码进行译码
			io_timer_free_channel(channel);  //如果选中定时器，则释放通道
			current &= ~(1 << channel);
		}
	}

    // Now allocate the new set应用新的设置

	for (unsigned channel = 0; channel_mask != 0 &&  channel < MAX_TIMER_IO_CHANNELS; channel++) {  //通道掩码不为0
		if (channel_mask & (1 << channel)) {//对每个通道进行扫描

			// First free any that were not PWM mode before

			if (-EBUSY == io_timer_is_channel_free(channel)) {  //判断通道是不是空闲的
				io_timer_free_channel(channel);  //释放通道
			}

			io_timer_channel_init(channel, IOTimerChanMode_PWMOut, NULL, NULL);  //对通道进行初始化
			channel_mask &= ~(1 << channel);  //对通道掩码进行清零操作
		}
	}

	return OK;
}

void up_pwm_servo_deinit(void)
{
	/* disable the timers */
	up_pwm_servo_arm(false);
}

int up_pwm_servo_set_rate_group_update(unsigned group, unsigned rate)
{
	/* limit update rate to 1..10000Hz; somewhat arbitrary but safe */
	if (rate < 1) {
		return -ERANGE;
	}

	if (rate > 10000) {
		return -ERANGE;
	}

	if ((group >= MAX_IO_TIMERS) || (io_timers[group].base == 0)) {
		return ERROR;
	}

	io_timer_set_rate(group, rate);

	return OK;
}

int up_pwm_servo_set_rate(unsigned rate)
{
	for (unsigned i = 0; i < MAX_IO_TIMERS; i++) {
		up_pwm_servo_set_rate_group_update(i, rate);
	}

	return 0;
}

uint32_t up_pwm_servo_get_rate_group(unsigned group)
{
	return io_timer_get_group(group);
}

void
up_pwm_servo_arm(bool armed)
{
	io_timer_set_enable(armed, IOTimerChanMode_PWMOut, IO_TIMER_ALL_MODES_CHANNELS);
}
