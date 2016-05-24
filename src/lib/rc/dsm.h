/**
 * @file dsm.h
 *
 * RC protocol definition for Spektrum RC
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include <stdint.h>
#include <nuttx/config.h>
#include <board_config.h>
#include <px4_defines.h>

__BEGIN_DECLS

#define DSM_FRAME_SIZE		16		/**< DSM frame size in bytes */
#define DSM_FRAME_CHANNELS	7		/**< Max supported DSM channels per frame */
#define DSM_MAX_CHANNEL_COUNT   18  /**< Max channel count of any DSM RC */
#define DSM_BUFFER_SIZE		(DSM_FRAME_SIZE + DSM_FRAME_SIZE / 2)

__EXPORT int	dsm_init(const char *device);
__EXPORT int	dsm_config(int dsm_fd);
__EXPORT bool	dsm_input(int dsm_fd, uint16_t *values, uint16_t *num_values, bool *dsm_11_bit, uint8_t *n_bytes,
			  uint8_t **bytes, unsigned max_values);

__EXPORT bool	dsm_parse(uint64_t now, uint8_t *frame, unsigned len, uint16_t *values,
			  uint16_t *num_values, bool *dsm_11_bit, unsigned *frame_drops, uint16_t max_channels);

#ifdef GPIO_SPEKTRUM_PWR_EN
__EXPORT void	dsm_bind(uint16_t cmd, int pulses);
#endif

enum DSM_CMD {							/* DSM bind states */
	DSM_CMD_BIND_POWER_DOWN = 0,
	DSM_CMD_BIND_POWER_UP,
	DSM_CMD_BIND_SET_RX_OUT,
	DSM_CMD_BIND_SEND_PULSES,
	DSM_CMD_BIND_REINIT_UART
};

__END_DECLS
