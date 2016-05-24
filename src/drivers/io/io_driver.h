
/**
 * @file px4io_driver.h
 *
 * Interface for PX4IO
 */

#pragma once

#include <board_config.h>

#ifdef PX4_I2C_OBDEV_PX4IO
device::Device	*PX4IO_i2c_interface();
#endif

#ifdef PX4IO_SERIAL_BASE
device::Device	*PX4IO_serial_interface();
#endif
