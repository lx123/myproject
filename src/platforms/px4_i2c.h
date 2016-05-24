/**
 * @file px4_i2c.h
 *
 * Includes device headers depending on the build target
 */

#pragma once

#define PX4_I2C_M_READ           0x0001          /* read data, from slave to master */


#include <sys/ioctl.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
//#include <nuttx/i2c/i2c_master.h>
#include <stm32.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <chip.h>
#include <arch/board/board.h>
#include <arch/chip/chip.h>
#include "up_internal.h"
#include "up_arch.h"

#define px4_i2c_msg_t i2c_msg_s
//#define px4_i2c_config_s i2c_config_s

//typedef struct i2c_dev_s px4_i2c_dev_t;
typedef struct i2c_master_s px4_i2c_dev_t;



