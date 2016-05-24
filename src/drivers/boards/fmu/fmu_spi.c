/**
 * @file px4fmu_spi.c
 *
 * Board-specific SPI functions.
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include <up_arch.h>
#include <chip.h>
#include <stm32.h>
#include "board_config.h"


/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ************************************************************************************/

__EXPORT void stm32_spiinitialize(void)
{
#ifdef CONFIG_STM32_SPI1  //板上的传感器都挂在spi总线上
	stm32_configgpio(GPIO_SPI_CS_GYRO);
	stm32_configgpio(GPIO_SPI_CS_ACCEL_MAG);
	stm32_configgpio(GPIO_SPI_CS_BARO);
	stm32_configgpio(GPIO_SPI_CS_HMC);
	stm32_configgpio(GPIO_SPI_CS_MPU);

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	stm32_gpiowrite(GPIO_SPI_CS_GYRO, 1);
	stm32_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
	stm32_gpiowrite(GPIO_SPI_CS_BARO, 1);
	stm32_gpiowrite(GPIO_SPI_CS_HMC, 1);
	stm32_gpiowrite(GPIO_SPI_CS_MPU, 1);

	stm32_configgpio(GPIO_EXTI_GYRO_DRDY);
	stm32_configgpio(GPIO_EXTI_MAG_DRDY);
	stm32_configgpio(GPIO_EXTI_ACCEL_DRDY);
	stm32_configgpio(GPIO_EXTI_MPU_DRDY);
#endif

#ifdef CONFIG_STM32_SPI2 //FRAM挂在spi2上
	stm32_configgpio(GPIO_SPI_CS_FRAM);
	stm32_gpiowrite(GPIO_SPI_CS_FRAM, 1);
#endif

#ifdef CONFIG_STM32_SPI4
	stm32_configgpio(GPIO_SPI_CS_EXT0);
	stm32_configgpio(GPIO_SPI_CS_EXT1);
	stm32_configgpio(GPIO_SPI_CS_EXT2);
	stm32_configgpio(GPIO_SPI_CS_EXT3);
	stm32_gpiowrite(GPIO_SPI_CS_EXT0, 1);
	stm32_gpiowrite(GPIO_SPI_CS_EXT1, 1);
	stm32_gpiowrite(GPIO_SPI_CS_EXT2, 1);
	stm32_gpiowrite(GPIO_SPI_CS_EXT3, 1);
#endif
}

//对spi1上挂载的传感器进行选择
__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	switch (devid) {
	case PX4_SPIDEV_GYRO:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_GYRO, !selected);//每次只选1个
		stm32_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
		stm32_gpiowrite(GPIO_SPI_CS_BARO, 1);
		stm32_gpiowrite(GPIO_SPI_CS_HMC, 1);
		stm32_gpiowrite(GPIO_SPI_CS_MPU, 1);
		break;

	case PX4_SPIDEV_ACCEL_MAG:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_GYRO, 1);
		stm32_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, !selected);
		stm32_gpiowrite(GPIO_SPI_CS_BARO, 1);
		stm32_gpiowrite(GPIO_SPI_CS_HMC, 1);
		stm32_gpiowrite(GPIO_SPI_CS_MPU, 1);
		break;

	case PX4_SPIDEV_BARO:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_GYRO, 1);
		stm32_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
		stm32_gpiowrite(GPIO_SPI_CS_BARO, !selected);
		stm32_gpiowrite(GPIO_SPI_CS_HMC, 1);
		stm32_gpiowrite(GPIO_SPI_CS_MPU, 1);
		break;

	case PX4_SPIDEV_HMC:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_GYRO, 1);
		stm32_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
		stm32_gpiowrite(GPIO_SPI_CS_BARO, 1);
		stm32_gpiowrite(GPIO_SPI_CS_HMC, !selected);
		stm32_gpiowrite(GPIO_SPI_CS_MPU, 1);
		break;

	case PX4_SPIDEV_MPU:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_GYRO, 1);
		stm32_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
		stm32_gpiowrite(GPIO_SPI_CS_BARO, 1);
		stm32_gpiowrite(GPIO_SPI_CS_HMC, 1);
		stm32_gpiowrite(GPIO_SPI_CS_MPU, !selected);
		break;

	default:
		break;
	}
}

//查询spi1状态
__EXPORT uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	return SPI_STATUS_PRESENT;
}


#ifdef CONFIG_STM32_SPI2
__EXPORT void stm32_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* there can only be one device on this bus, so always select it */
	stm32_gpiowrite(GPIO_SPI_CS_FRAM, !selected);
}

__EXPORT uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	/* FRAM is always present */
	return SPI_STATUS_PRESENT;
}
#endif

__EXPORT void stm32_spi4select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	switch (devid) {
	case PX4_SPIDEV_EXT0:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_EXT0, !selected);
		stm32_gpiowrite(GPIO_SPI_CS_EXT1, 1);
		stm32_gpiowrite(GPIO_SPI_CS_EXT2, 1);
		stm32_gpiowrite(GPIO_SPI_CS_EXT3, 1);
		break;

	case PX4_SPIDEV_EXT1:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_EXT0, 1);
		stm32_gpiowrite(GPIO_SPI_CS_EXT1, !selected);
		stm32_gpiowrite(GPIO_SPI_CS_EXT2, 1);
		stm32_gpiowrite(GPIO_SPI_CS_EXT3, 1);
		break;

	case PX4_SPIDEV_EXT2:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_EXT0, 1);
		stm32_gpiowrite(GPIO_SPI_CS_EXT1, 1);
		stm32_gpiowrite(GPIO_SPI_CS_EXT2, !selected);
		stm32_gpiowrite(GPIO_SPI_CS_EXT3, 1);
		break;

	case PX4_SPIDEV_EXT3:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_EXT0, 1);
		stm32_gpiowrite(GPIO_SPI_CS_EXT1, 1);
		stm32_gpiowrite(GPIO_SPI_CS_EXT2, 1);
		stm32_gpiowrite(GPIO_SPI_CS_EXT3, !selected);
		break;

	default:
		break;

	}
}

//查询spi4当前状态
__EXPORT uint8_t stm32_spi4status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	return SPI_STATUS_PRESENT;
}


