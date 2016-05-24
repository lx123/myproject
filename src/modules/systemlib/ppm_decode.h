
/**
 * @file ppm_decode.h
 *
 * PPM input decoder.
 */

#pragma once

#include <drivers/drv_hrt.h>

/**
 * Maximum number of channels that we will decode.
 */
#define PPM_MAX_CHANNELS	12

/* PPM input nominal min/max values */
#define PPM_MIN 1000
#define PPM_MAX 2000
#define PPM_MID ((PPM_MIN + PPM_MAX) / 2)

__BEGIN_DECLS

/*
 * PPM decoder state
 */
__EXPORT extern uint16_t	ppm_buffer[PPM_MAX_CHANNELS];	/**< decoded PPM channel values */
__EXPORT extern uint16_t	ppm_frame_length;				/**< length of the decoded PPM frame (includes gap) */
__EXPORT extern unsigned	ppm_decoded_channels;	/**< count of decoded channels */
__EXPORT extern hrt_abstime	ppm_last_valid_decode;	/**< timestamp of the last valid decode */

/**
 * Initialise the PPM input decoder.
 *
 * @param count_max		The maximum value of the counter passed to
 *				ppm_input_decode, used to determine how to
 *				handle counter wrap.
 */
__EXPORT void		ppm_input_init(unsigned count_max);

/**
 * Inform the decoder of an edge on the PPM input.
 *
 * This function can be registered with the HRT as the PPM edge handler.
 *
 * @param reset			If set, the edge detector has missed one or
 *				more edges and the decoder needs to be reset.
 * @param count			A microsecond timestamp corresponding to the
 *				edge, in the range 0-count_max.  This value
 *				is expected to wrap.
 */
__EXPORT void		ppm_input_decode(bool reset, unsigned count);

__END_DECLS
