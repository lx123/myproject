
/**
 * @file conversions.h
 * Definition of commonly used conversions.
 *
 * Includes bit / byte / geo representation and unit conversions.
 */

#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_
#include <float.h>
#include <stdint.h>

__BEGIN_DECLS

/**
 * Converts a signed 16 bit integer from big endian to little endian.
 *
 * This function is for commonly used 16 bit big endian sensor data,
 * delivered by driver routines as two 8 bit numbers in big endian order.
 * Common vendors with big endian representation are Invense, Bosch and
 * Honeywell. ST micro devices tend to use a little endian representation.
 */
__EXPORT int16_t int16_t_from_bytes(uint8_t bytes[]);

__END_DECLS

#endif /* CONVERSIONS_H_ */


