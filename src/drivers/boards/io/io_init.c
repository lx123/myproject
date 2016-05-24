
/**
 * @file px4iov2_init.c
 *
 * PX4FMU-specific early startup code.  This file implements the
 * nsh_archinitialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialisation.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>

#include <stm32.h>
#include "board_config.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#  else
#    define message printf
#  endif
#endif

/****************************************************************************
 * Protected Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void stm32_boardinitialize(void)  //io口初始化
{

	/* configure GPIOs */

	/* LEDS - default to off */
	stm32_configgpio(GPIO_LED1);  //蓝色
	stm32_configgpio(GPIO_LED2);  //琥珀
	stm32_configgpio(GPIO_LED3);   //安全开关
	stm32_configgpio(GPIO_LED4);   //空余

	stm32_configgpio(GPIO_BTN_SAFETY);  //安全开关按钮

	/* spektrum power enable is active high - enable it by default */
	stm32_configgpio(GPIO_SPEKTRUM_PWR_EN);

	stm32_configgpio(GPIO_SERVO_FAULT_DETECT);

	/* RSSI inputs */   //接收机强度输入
	stm32_configgpio(GPIO_TIM_RSSI); /* xxx alternate function */
	stm32_configgpio(GPIO_ADC_RSSI);

	/* servo rail voltage */
	stm32_configgpio(GPIO_ADC_VSERVO);  //伺服传感器电压

	stm32_configgpio(GPIO_SBUS_INPUT); /* xxx alternate function */
	stm32_configgpio(GPIO_SBUS_OUTPUT);

	/* sbus output enable is active low - disable it by default */
	stm32_gpiowrite(GPIO_SBUS_OENABLE, true);
	stm32_configgpio(GPIO_SBUS_OENABLE);

	stm32_configgpio(GPIO_PPM); /* xxx alternate function */

	stm32_gpiowrite(GPIO_PWM1, true);
	stm32_configgpio(GPIO_PWM1);

	stm32_gpiowrite(GPIO_PWM2, true);
	stm32_configgpio(GPIO_PWM2);

	stm32_gpiowrite(GPIO_PWM3, true);
	stm32_configgpio(GPIO_PWM3);

	stm32_gpiowrite(GPIO_PWM4, true);
	stm32_configgpio(GPIO_PWM4);

	stm32_gpiowrite(GPIO_PWM5, true);
	stm32_configgpio(GPIO_PWM5);

	stm32_gpiowrite(GPIO_PWM6, true);
	stm32_configgpio(GPIO_PWM6);

	stm32_gpiowrite(GPIO_PWM7, true);
	stm32_configgpio(GPIO_PWM7);

	stm32_gpiowrite(GPIO_PWM8, true);
	stm32_configgpio(GPIO_PWM8);
}
