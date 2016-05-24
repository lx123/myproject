
/**
 * @file board_config.h
 *
 * PX4IOV2 internal definitions
 */

#pragma once

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/* these headers are not C++ safe */
#include <stm32.h>
#include <arch/board/board.h>

/******************************************************************************
 * Definitions
 ******************************************************************************/
/* Configuration **************************************************************/

/******************************************************************************
 * Serial
 ******************************************************************************/
#define PX4FMU_SERIAL_BASE	STM32_USART2_BASE       //串口2基地址
#define PX4FMU_SERIAL_VECTOR	STM32_IRQ_USART2    //串口2中断
#define PX4FMU_SERIAL_TX_GPIO	GPIO_USART2_TX      //发送io
#define PX4FMU_SERIAL_RX_GPIO	GPIO_USART2_RX      //接收io
#define PX4FMU_SERIAL_TX_DMA	DMACHAN_USART2_TX  //dma通道
#define PX4FMU_SERIAL_RX_DMA	DMACHAN_USART2_RX
#define PX4FMU_SERIAL_CLOCK	STM32_PCLK1_FREQUENCY  //串口时钟速率
#define PX4FMU_SERIAL_BITRATE	1500000            //串口速率

/******************************************************************************
 * GPIOS
 ******************************************************************************/

/* LEDS  **********************************************************************/

#define GPIO_LED1 (GPIO_OUTPUT|GPIO_CNF_OUTOD|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN14)   //蓝灯
#define GPIO_LED2 (GPIO_OUTPUT|GPIO_CNF_OUTOD|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN15)   //琥珀灯
#define GPIO_LED3 (GPIO_OUTPUT|GPIO_CNF_OUTOD|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN13)   //安全开关
#define GPIO_LED4 (GPIO_OUTPUT|GPIO_CNF_OUTOD|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN11)  //多余的pwm

#define GPIO_USART1_RX_SPEKTRUM		(GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN10)   //SPEKTRUM	协议接口

/* Safety switch button *******************************************************/

#define GPIO_BTN_SAFETY (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN5)  //安全开关按钮

/* Power switch controls ******************************************************/

#define GPIO_SPEKTRUM_PWR_EN (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)  //SPEKTRUM电源使能
#define POWER_SPEKTRUM(_s)		stm32_gpiowrite(GPIO_SPEKTRUM_PWR_EN, (_s))

#define SPEKTRUM_RX_HIGH(_s)	stm32_gpiowrite(GPIO_USART1_RX_SPEKTRUM, (_s))
#define SPEKTRUM_RX_AS_UART()		stm32_configgpio(GPIO_USART1_RX)  //接收作为串口
#define SPEKTRUM_RX_AS_GPIO()		stm32_configgpio(GPIO_USART1_RX_SPEKTRUM)  //接收作为io口

#define GPIO_SERVO_FAULT_DETECT (GPIO_INPUT|GPIO_CNF_INPULLUP|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)

/* Analog inputs **************************************************************/

#define GPIO_ADC_VSERVO (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN4)  //伺服传感器

/* the same rssi signal goes to both an adc and a timer input */
#define GPIO_ADC_RSSI   (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN5)
#define GPIO_TIM_RSSI   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN12)

/* PWM pins  **************************************************************/

#define GPIO_PPM (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN8)  //ppwm输入

#define GPIO_PWM1 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN0)
#define GPIO_PWM2 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)
#define GPIO_PWM3 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN8)
#define GPIO_PWM4 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN9)
#define GPIO_PWM5 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN6)
#define GPIO_PWM6 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN7)
#define GPIO_PWM7 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)
#define GPIO_PWM8 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)

/* SBUS pins  *************************************************************/

/* XXX these should be UART pins */
#define GPIO_SBUS_INPUT   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN11)
#define GPIO_SBUS_OUTPUT  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN10)
#define GPIO_SBUS_OENABLE (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN4)

/*
 * High-resolution timer
 */
#define HRT_TIMER		1	/* use timer1 for the HRT */    //定时器1作为高精度定时器
#define HRT_TIMER_CHANNEL	2	/* use capture/compare channel 2 */   //使用捕捉/比较通道2
#define HRT_PPM_CHANNEL		1	/* use capture/compare channel 1 */
#define GPIO_PPM_IN		(GPIO_ALT|GPIO_CNF_INPULLUP|GPIO_PORTE|GPIO_PIN9)  //ppm输入

/* LED definitions ******************************************************************/
/* PX4 has two LEDs that we will encode as: */

#define LED_STARTED       0  /* LED? */
#define LED_HEAPALLOCATE  1  /* LED? */
#define LED_IRQSENABLED   2  /* LED? + LED? */
#define LED_STACKCREATED  3  /* LED? */
#define LED_INIRQ         4  /* LED? + LED? */
#define LED_SIGNAL        5  /* LED? + LED? */
#define LED_ASSERTION     6  /* LED? + LED? + LED? */
#define LED_PANIC         7  /* N/C  + N/C  + N/C + LED? */

