/**
 * @file err.h
 *
 * Simple error/warning functions, heavily inspired by the BSD functions of
 * the same names.
 *
 * The err() and warn() family of functions display a formatted error
 * message on the standard error output.  In all cases, the last
 * component of the program name, a colon character, and a space are
 * output.  If the fmt argument is not NULL, the printf(3)-like formatted
 * error message is output.  The output is terminated by a newline
 * character.
 *
 * The err(), errc(), verr(), verrc(), warn(), warnc(), vwarn(), and
 * vwarnc() functions append an error message obtained from strerror(3)
 * based on a supplied error code value or the global variable errno,
 * preceded by another colon and space unless the fmt argument is NULL.
 *
 * In the case of the errc(), verrc(), warnc(), and vwarnc() functions,
 * the code argument is used to look up the error message.
 *
 * The err(), verr(), warn(), and vwarn() functions use the global
 * variable errno to look up the error message.
 *
 * The errx() and warnx() functions do not append an error message.
 *
 * The err(), verr(), errc(), verrc(), errx(), and verrx() functions do
 * not return, but exit with the value of the argument eval.
 *
 */
#ifndef _SYSTEMLIB_ERR_H
#define _SYSTEMLIB_ERR_H

#include <px4_log.h>
#include <stdarg.h>
#include "visibility.h"

__BEGIN_DECLS

__EXPORT const char *getprogname(void);
__EXPORT void	err(int eval, const char *fmt, ...)		__attribute__((noreturn, format(printf, 2, 3)));
__EXPORT void	verr(int eval, const char *fmt, va_list)	__attribute__((noreturn, format(printf, 2, 0)));
__EXPORT void	errc(int eval, int code, const char *fmt, ...)	__attribute__((noreturn, format(printf, 3, 4)));
__EXPORT void	verrc(int eval, int code, const char *fmt, va_list) __attribute__((noreturn, format(printf, 3, 0)));
__EXPORT void	errx(int eval, const char *fmt, ...) 		__attribute__((noreturn, format(printf, 2, 3)));
__EXPORT void	verrx(int eval, const char *fmt, va_list)	__attribute__((noreturn, format(printf, 2, 0)));
__EXPORT void	warn(const char *fmt, ...)			__attribute__((format(printf, 1, 2)));
__EXPORT void	vwarn(const char *fmt, va_list)			__attribute__((format(printf, 1, 0)));
__EXPORT void	warnc(int code, const char *fmt, ...)		__attribute__((format(printf, 2, 3)));
__EXPORT void	vwarnc(int code, const char *fmt, va_list)	__attribute__((format(printf, 2, 0)));
__EXPORT void	warnx(const char *fmt, ...)			__attribute__((format(printf, 1, 2)));
__EXPORT void	vwarnx(const char *fmt, va_list)		__attribute__((format(printf, 1, 0)));

__END_DECLS

#endif










