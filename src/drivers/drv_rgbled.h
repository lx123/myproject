/**
 * @file drv_rgbled.h
 *
 * RGB led device API
 */

#pragma once

#include <stdint.h>
#include <sys/ioctl.h>

#define RGBLED_BASE_DEVICE_PATH "/dev/rgbled"

/* more devices will be 1, 2, etc */
#define RGBLED0_DEVICE_PATH "/dev/rgbled0"

/*
 * ioctl() definitions
 */

#define _RGBLEDIOCBASE		(0x2900) //基地址
#define _RGBLEDIOC(_n)		(_PX4_IOC(_RGBLEDIOCBASE, _n))

/** play the named script in *(char *)arg, repeating forever */
#define RGBLED_PLAY_SCRIPT_NAMED	_RGBLEDIOC(1)

/** play the numbered script in (arg), repeating forever */
#define RGBLED_PLAY_SCRIPT		_RGBLEDIOC(2)


/**
 * Set the user script; (arg) is a pointer to an array of script lines,
 * where each line is an array of four bytes giving <duration>, <command>, arg[0-2]
 *
 * The script is terminated by a zero command.
 */
#define RGBLED_SET_USER_SCRIPT		_RGBLEDIOC(3)

/** set constant RGB values */
#define RGBLED_SET_RGB			_RGBLEDIOC(4)

/** set color */
#define RGBLED_SET_COLOR		_RGBLEDIOC(5)

/** set blink speed */
#define RGBLED_SET_MODE			_RGBLEDIOC(6)

/** set pattern */
#define RGBLED_SET_PATTERN		_RGBLEDIOC(7)

/*
  structure passed to RGBLED_SET_RGB ioctl()
  Note that the driver scales the brightness to 0 to 255, regardless
  of the hardware scaling
 */
typedef struct {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} rgbled_rgbset_t;

/* enum passed to RGBLED_SET_COLOR ioctl()*/
typedef enum {
	RGBLED_COLOR_OFF,
	RGBLED_COLOR_RED,
	RGBLED_COLOR_YELLOW,
	RGBLED_COLOR_PURPLE,
	RGBLED_COLOR_GREEN,
	RGBLED_COLOR_BLUE,
	RGBLED_COLOR_WHITE,
	RGBLED_COLOR_AMBER,
	RGBLED_COLOR_DIM_RED,
	RGBLED_COLOR_DIM_YELLOW,
	RGBLED_COLOR_DIM_PURPLE,
	RGBLED_COLOR_DIM_GREEN,
	RGBLED_COLOR_DIM_BLUE,
	RGBLED_COLOR_DIM_WHITE,
	RGBLED_COLOR_DIM_AMBER
} rgbled_color_t;


/* enum passed to RGBLED_SET_MODE ioctl()*/
typedef enum {
	RGBLED_MODE_OFF,
	RGBLED_MODE_ON,
	RGBLED_MODE_BLINK_SLOW,
	RGBLED_MODE_BLINK_NORMAL,
	RGBLED_MODE_BLINK_FAST,
	RGBLED_MODE_BREATHE,
	RGBLED_MODE_PATTERN
} rgbled_mode_t;

#define RGBLED_PATTERN_LENGTH 20  //模式长度

typedef struct {
	rgbled_color_t color[RGBLED_PATTERN_LENGTH];
	unsigned duration[RGBLED_PATTERN_LENGTH];
} rgbled_pattern_t;















