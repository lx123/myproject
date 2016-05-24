/*
 * @file px4iov2_timer_config.c
 *
 * Configuration data for the stm32 pwm_servo driver.
 *
 * Note that these arrays must always be fully-sized.
 */

#include <stdint.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/stm32/drv_io_timer.h>

#include <arch/board/board.h>

#include <stm32.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

__EXPORT const io_timers_t io_timers[MAX_IO_TIMERS] = {
	{
		.base = STM32_TIM2_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM2EN,
		.clock_freq = STM32_APB1_TIM2_CLKIN,
		.first_channel_index = 0,
		.last_channel_index = 1,
		.handler = io_timer_handler0,
		.vectorno =  STM32_IRQ_TIM2,
	},
	{
		.base = STM32_TIM3_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM3EN,
		.clock_freq = STM32_APB1_TIM3_CLKIN,
		.first_channel_index = 4,
		.last_channel_index = 7,
		.handler = io_timer_handler1,
		.vectorno =  STM32_IRQ_TIM3,
	},
	{
		.base = STM32_TIM4_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM4EN,
		.clock_freq = STM32_APB1_TIM4_CLKIN,
		.first_channel_index = 2,
		.last_channel_index = 3,
		.handler = io_timer_handler2,
		.vectorno =  STM32_IRQ_TIM4,
	}
};

__EXPORT const timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	{
		.gpio_out = GPIO_TIM2_CH1OUT,
		.gpio_in = GPIO_TIM2_CH1IN,
		.timer_index = 0,
		.timer_channel = 1,
		.ccr_offset = STM32_GTIM_CCR1_OFFSET,
		.masks  = GTIM_SR_CC1IF | GTIM_SR_CC1OF
	},
	{
		.gpio_out = GPIO_TIM2_CH2OUT,
		.gpio_in = GPIO_TIM2_CH2IN,
		.timer_index = 0,
		.timer_channel = 2,
		.ccr_offset = STM32_GTIM_CCR2_OFFSET,
		.masks  = GTIM_SR_CC2IF | GTIM_SR_CC2OF
	},
	{
		.gpio_out = GPIO_TIM4_CH3OUT,
		.gpio_in = GPIO_TIM4_CH3IN,
		.timer_index = 2,
		.timer_channel = 3,
		.ccr_offset = STM32_GTIM_CCR3_OFFSET,
		.masks  = GTIM_SR_CC3IF | GTIM_SR_CC3OF
	},
	{
		.gpio_out = GPIO_TIM4_CH4OUT,
		.gpio_in = GPIO_TIM4_CH4IN,
		.timer_index = 2,
		.timer_channel = 4,
		.ccr_offset = STM32_GTIM_CCR4_OFFSET,
		.masks  = GTIM_SR_CC4IF | GTIM_SR_CC4OF
	},
	{
		.gpio_out = GPIO_TIM3_CH1OUT,
		.gpio_in = GPIO_TIM3_CH1IN,
		.timer_index = 1,
		.timer_channel = 1,
		.ccr_offset = STM32_GTIM_CCR1_OFFSET,
		.masks  = GTIM_SR_CC1IF | GTIM_SR_CC1OF
	},
	{
		.gpio_out = GPIO_TIM3_CH2OUT,
		.gpio_in = GPIO_TIM3_CH2IN,
		.timer_index = 1,
		.timer_channel = 2,
		.ccr_offset = STM32_GTIM_CCR2_OFFSET,
		.masks  = GTIM_SR_CC2IF | GTIM_SR_CC2OF
	},
	{
		.gpio_out = GPIO_TIM3_CH3OUT,
		.gpio_in = GPIO_TIM3_CH3IN,
		.timer_index = 1,
		.timer_channel = 3,
		.ccr_offset = STM32_GTIM_CCR3_OFFSET,
		.masks  = GTIM_SR_CC3IF | GTIM_SR_CC3OF
	},
	{
		.gpio_out = GPIO_TIM3_CH4OUT,
		.gpio_in = GPIO_TIM3_CH4IN,
		.timer_index = 1,
		.timer_channel = 4,
		.ccr_offset = STM32_GTIM_CCR4_OFFSET,
		.masks  = GTIM_SR_CC4IF | GTIM_SR_CC4OF
	}
};
