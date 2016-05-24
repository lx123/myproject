/**
 * @file drv_hrt.h
 *
 * High-resolution timer with callouts and timekeeping.
 */


#pragma once

#include <sys/types.h>
#include <stdbool.h>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include <px4_time.h>
#include <queue.h>

__BEGIN_DECLS

/**
 * Absolute time, in microsecond units.
 *
 * Absolute time is measured from some arbitrary epoch shortly after
 * system startup.  It should never wrap or go backwards.
 */
typedef uint64_t	hrt_abstime;  //绝对时间，单位us

/**
 * Callout function type.
 *
 * Note that callouts run in the timer interrupt context, so
 * they are serialised with respect to each other, and must not
 * block.
 */
typedef void	(* hrt_callout)(void *arg);  //运行在定时器中断环境

/**
 * Callout record.调用记录
 */
typedef struct hrt_call {
	struct sq_entry_s	link;

	hrt_abstime		deadline;
	hrt_abstime		period;
	hrt_callout		callout;
	void			*arg;
} *hrt_call_t;

/**
 * Get absolute time.  获得绝对时间
 */
__EXPORT extern hrt_abstime hrt_absolute_time(void);

/**
 * Convert a timespec to absolute time. 转换一个标准时间到绝对时间
 */
__EXPORT extern hrt_abstime ts_to_abstime(struct timespec *ts);

/**
 * Convert absolute time to a timespec.  转换绝对时间到标准时间
 */
__EXPORT extern void	abstime_to_ts(struct timespec *ts, hrt_abstime abstime);

/**
 * Compute the delta between a timestamp taken in the past
 * and now.
 *
 * This function is safe to use even if the timestamp is updated
 * by an interrupt during execution.
 */
__EXPORT extern hrt_abstime hrt_elapsed_time(const volatile hrt_abstime *then);  //计算两个时间戳之差

/**
 * Store the absolute time in an interrupt-safe fashion.
 *
 * This function ensures that the timestamp cannot be seen half-written by an interrupt handler.
 */
__EXPORT extern hrt_abstime hrt_store_absolute_time(volatile hrt_abstime *now); //保存绝对时间在一个中断保护下

/**
 * Call callout(arg) after delay has elapsed.
 *
 * If callout is NULL, this can be used to implement a timeout by testing the call
 * with hrt_called().
 */
__EXPORT extern void	hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg);

/**
 * Call callout(arg) at absolute time calltime. 调用一个回调函数在一个绝对的调用时间
 */
__EXPORT extern void	hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg);

/**
 * Call callout(arg) after delay, and then after every interval.
 *
 * Note thet the interval is timed between scheduled, not actual, call times, so the call rate may
 * jitter but should not drift.
 */
__EXPORT extern void	hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval,  //每个一段时间调用一个回调函数
				       hrt_callout callout, void *arg);

/**
 * If this returns true, the entry has been invoked and removed from the callout list,
 * or it has never been entered.
 *
 * Always returns false for repeating callouts.
 */
__EXPORT extern bool	hrt_called(struct hrt_call *entry);

/**
 * Remove the entry from the callout list.
 */
__EXPORT extern void	hrt_cancel(struct hrt_call *entry);  //在回调列表中取消一个入口

/**
 * Initialise a hrt_call structure
 */
__EXPORT extern void	hrt_call_init(struct hrt_call *entry);  //初始化一个回调结构

/*
 * delay a hrt_call_every() periodic call by the given number of
 * microseconds. This should be called from within the callout to
 * cause the callout to be re-scheduled for a later time. The periodic
 * callouts will then continue from that new base time at the
 * previously specified period.
 */
__EXPORT extern void	hrt_call_delay(struct hrt_call *entry, hrt_abstime delay);  //

/*
 * Initialise the HRT.
 */
__EXPORT extern void	hrt_init(void);  //初始化高精度定时器

__END_DECLS





