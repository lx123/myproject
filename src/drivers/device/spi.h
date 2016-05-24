
/**
 * @file spi.h
 *
 * Base class for devices connected via SPI.
 */

#ifndef _DEVICE_SPI_H
#define _DEVICE_SPI_H

#include "device_nuttx.h"

#include <stm32.h>

namespace device __EXPORT
{
	/**
	 * Abstract class for character device on SPI
	 */
class __EXPORT SPI : public CDev
{
protected:
	/**
	 * Constructor
	 *
	 * @param name		Driver name
	 * @param devname	Device node name
	 * @param bus		SPI bus on which the device lives
	 * @param device	Device handle (used by SPI_SELECT)
	 * @param mode		SPI clock/data mode
	 * @param frequency	SPI clock frequency
	 * @param irq		Interrupt assigned to the device (or zero if none)
	 */
	SPI(const char *name,
	    const char *devname,
	    int bus,
	    enum spi_dev_e device,
	    enum spi_mode_e mode,
	    uint32_t frequency,
	    int irq = 0);
	virtual ~SPI();

	/**
	 * Locking modes supported by the driver.
	 */
	enum LockMode {
		LOCK_PREEMPTION,	/**< the default; lock against all forms of preemption. */
		LOCK_THREADS,		/**< lock only against other threads, using SPI_LOCK */
		LOCK_NONE		/**< perform no locking, only safe if the bus is entirely private */
	};

	virtual int	init();

	/**
	 * Check for the presence of the device on the bus.
	 */
	virtual int	probe(); //检查当前在总线上的设备

	/**
	 * Perform a SPI transfer.
	 *
	 * If called from interrupt context, this interface does not lock
	 * the bus and may interfere with non-interrupt-context callers.
	 *
	 * Clients in a mixed interrupt/non-interrupt configuration must
	 * ensure appropriate interlocking.
	 *
	 * At least one of send or recv must be non-null.
	 *
	 * @param send		Bytes to send to the device, or nullptr if
	 *			no data is to be sent.
	 * @param recv		Buffer for receiving bytes from the device,
	 *			or nullptr if no bytes are to be received.
	 * @param len		Number of bytes to transfer.
	 * @return		OK if the exchange was successful, -errno
	 *			otherwise.
	 */
	int		transfer(uint8_t *send, uint8_t *recv, unsigned len);

	/**
	 * Set the SPI bus frequency
	 * This is used to change frequency on the fly. Some sensors
	 * (such as the MPU6000) need a lower frequency for setup
	 * registers and can handle higher frequency for sensor
	 * value registers
	 *
	 * @param frequency	Frequency to set (Hz)
	 */
	void		set_frequency(uint32_t frequency);

	/**
	 * Set the SPI bus locking mode
	 *
	 * This set the SPI locking mode. For devices competing with NuttX SPI
	 * drivers on a bus the right lock mode is LOCK_THREADS.
	 *
	 * @param mode	Locking mode
	 */
	void		set_lockmode(enum LockMode mode) { locking_mode = mode; }

	LockMode	locking_mode;	/**< selected locking mode */

private:
	enum spi_dev_e		_device;
	enum spi_mode_e		_mode;
	uint32_t		_frequency;
	struct spi_dev_s	*_dev;

	/* this class does not allow copying */
	SPI(const SPI &);
	SPI operator=(const SPI &);

protected:
	int			_bus;

	int	_transfer(uint8_t *send, uint8_t *recv, unsigned len);

};
} // namespace device

#endif /* _DEVICE_SPI_H */
