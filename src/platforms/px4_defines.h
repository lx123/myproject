/**
 * @file px4_defines.h
 *
 * Generally used magic defines
 */

#pragma once

#include <px4_log.h>
#include <math.h>

/* Get the name of the default value fiven the param name */
#define PX4_PARAM_DEFAULT_VALUE_NAME(_name) PARAM_##_name##_DEFAULT

/* Shortcuts to define parameters when the default value is defined according to PX4_PARAM_DEFAULT_VALUE_NAME */
#define PX4_PARAM_DEFINE_INT32(_name) PARAM_DEFINE_INT32(_name, PX4_PARAM_DEFAULT_VALUE_NAME(_name))
#define PX4_PARAM_DEFINE_FLOAT(_name) PARAM_DEFINE_FLOAT(_name, PX4_PARAM_DEFAULT_VALUE_NAME(_name))

#define PX4_ERROR (-1)
#define PX4_OK 0

/*
 * Building for NuttX or POSIX
 */
//#include <platforms/px4_includes.h>
/* Main entry point */
#define PX4_MAIN_FUNCTION(_prefix) int _prefix##_task_main(int argc, char *argv[])

/* Parameter handle datatype */
//#include <systemlib/param/param.h>
//typedef param_t px4_param_t;

/* Get value of parameter by name */
#define PX4_PARAM_GET_BYNAME(_name, _destpt) param_get(param_find(_name), _destpt)

/*
 * NuttX Specific defines
 */


#define PX4_ROOTFSDIR

/* XXX this is a hack to resolve conflicts with NuttX headers */
#if !defined(__PX4_TESTS)
#define isspace(c) \
	((c) == ' '  || (c) == '\t' || (c) == '\n' || \
	 (c) == '\r' || (c) == '\f' || c== '\v')
#endif

#define _PX4_IOC(x,y) _IOC(x,y)

#define px4_statfs_buf_f_bavail_t int

#define PX4_ISFINITE(x) isfinite(x)

// mode for open with O_CREAT
#define PX4_O_MODE_777 0777
#define PX4_O_MODE_666 0666
#define PX4_O_MODE_600 0600

#ifndef PRIu64
#define PRIu64 "llu"
#endif
#ifndef PRId64
#define PRId64 "lld"
#endif

/*
 *Defines for all platforms
 */

/* wrapper for 2d matrices */
#define PX4_ARRAY2D(_array, _ncols, _x, _y) (_array[_x * _ncols + _y])

/* wrapper for rotation matrices stored in arrays */
#define PX4_R(_array, _x, _y) PX4_ARRAY2D(_array, 3, _x, _y)







