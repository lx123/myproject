/**
 * @file drv_gpio.h
 *
 * Generic GPIO ioctl interface.
 */

#ifndef _DRV_GPIO_H
#define _DRV_GPIO_H

#include <sys/ioctl.h>

/*
 * PX4FMUv2 GPIO numbers.
 *
 * There are no alternate functions on this board.
 */
# define GPIO_SERVO_1		(1<<0)		/**< servo 1 output */
# define GPIO_SERVO_2		(1<<1)		/**< servo 2 output */
# define GPIO_SERVO_3		(1<<2)		/**< servo 3 output */
# define GPIO_SERVO_4		(1<<3)		/**< servo 4 output */
# define GPIO_SERVO_5		(1<<4)		/**< servo 5 output */
# define GPIO_SERVO_6		(1<<5)		/**< servo 6 output */

# define GPIO_5V_PERIPH_EN	(1<<6)		/**< PA8 - !VDD_5V_PERIPH_EN */
# define GPIO_3V3_SENSORS_EN	(1<<7)		/**< PE3 - VDD_3V3_SENSORS_EN */
# define GPIO_BRICK_VALID	(1<<8)		/**< PB5 - !VDD_BRICK_VALID */
# define GPIO_SERVO_VALID	(1<<9)		/**< PB7 - !VDD_SERVO_VALID */
# define GPIO_5V_HIPOWER_OC	(1<<10)		/**< PE10 - !VDD_5V_HIPOWER_OC */
# define GPIO_5V_PERIPH_OC	(1<<11)		/**< PE10 - !VDD_5V_PERIPH_OC */

/**
 * Device paths for things that support the GPIO ioctl protocol.
 */
# define PX4FMU_DEVICE_PATH	"/dev/px4fmu"
//# define PX4IO_DEVICE_PATH	"/dev/px4io"  //还没用到io板

/*
 * IOCTL definitions.  IO控制定义
 *
 * For all ioctls, the (arg) argument is a bitmask of GPIOs to be affected
 * by the operation, with the LSB being the lowest-numbered GPIO.
 *
 * Note that there may be board-specific relationships between GPIOs;
 * applications using GPIOs should be aware of this.
 */
#define _GPIOCBASE	0x2700  //端口c的基地址
#define GPIOC(_x)	_IOC(_GPIOCBASE, _x)  //直接使用C口

/** reset all board GPIOs to their default state */
#define GPIO_RESET	GPIOC(0)

/** configure the board GPIOs in (arg) as outputs */
#define GPIO_SET_OUTPUT	GPIOC(1)

/** configure the board GPIOs in (arg) as inputs */
#define GPIO_SET_INPUT	GPIOC(2)

/** configure the board GPIOs in (arg) for the first alternate function (if supported) */
#define GPIO_SET_ALT_1	GPIOC(3)

/** configure the board GPIO (arg) for the second alternate function (if supported) */
#define GPIO_SET_ALT_2	GPIOC(4)

/** configure the board GPIO (arg) for the third alternate function (if supported) */
#define GPIO_SET_ALT_3	GPIOC(5)

/** configure the board GPIO (arg) for the fourth alternate function (if supported) */
#define GPIO_SET_ALT_4	GPIOC(6)

/** set the GPIOs in (arg) */
#define GPIO_SET	GPIOC(10)

/** clear the GPIOs in (arg) */
#define GPIO_CLEAR	GPIOC(11)

/** read all the GPIOs and return their values in *(uint32_t *)arg */
#define GPIO_GET	GPIOC(12)

#define GPIO_SENSOR_RAIL_RESET	GPIOC(13)

#define GPIO_PERIPHERAL_RAIL_RESET	GPIOC(14)

#endif /* _DRV_GPIO_H */



