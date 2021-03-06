/**
 * @file fmu_led.c
 *
 * FMU LED backend.
 */
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "board_config.h"

/*
 * Ideally we'd be able to get these from up_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

__EXPORT void led_init()
{
	/* Configure LED1 GPIO for output */

	stm32_configgpio(GPIO_LED1);
}

__EXPORT void led_on(int led)
{
	if (led == 1) {
		/* Pull down to switch on */
		stm32_gpiowrite(GPIO_LED1, false);
	}
}

__EXPORT void led_off(int led)
{
	if (led == 1) {
		/* Pull up to switch off */
		stm32_gpiowrite(GPIO_LED1, true);
	}
}

__EXPORT void led_toggle(int led)
{
	if (led == 1) {
		if (stm32_gpioread(GPIO_LED1)) {
			stm32_gpiowrite(GPIO_LED1, false);

		} else {
			stm32_gpiowrite(GPIO_LED1, true);
		}
	}
}


