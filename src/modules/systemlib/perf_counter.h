/**
 * @file perf_counter.h
 * Performance measuring tools.
 */

#ifndef _SYSTEMLIB_PERF_COUNTER_H
#define _SYSTEMLIB_PERF_COUNTER_H value

#include <stdint.h>
#include <px4_defines.h>

/**
 * Counter types. 计数类型
 */
enum perf_counter_type {
	PC_COUNT,		/**< count the number of times an event occurs */  //事件计数
	PC_ELAPSED,		/**< measure the time elapsed performing an event */  //测量一个时间消耗的时间
	PC_INTERVAL		/**< measure the interval between instances of an event */  //测量两个时间发生的时间间隔
};


struct perf_ctr_header;
typedef struct perf_ctr_header	*perf_counter_t;

__BEGIN_DECLS

/**
 * Create a new local counter.
 *
 * @param type			The type of the new counter.
 * @param name			The counter name.
 * @return			Handle for the new counter, or NULL if a counter
 *				could not be allocated.
 */
__EXPORT extern perf_counter_t	perf_alloc(enum perf_counter_type type, const char *name);  //创建一个新的本地计数

/**
 * Get the reference to an existing counter or create a new one if it does not exist.
 *
 * @param type			The type of the counter.
 * @param name			The counter name.
 * @return			Handle for the counter, or NULL if a counter
 *				could not be allocated.
 */
__EXPORT extern perf_counter_t	perf_alloc_once(enum perf_counter_type type, const char *name); //获得参考的计数

/**
 * Free a counter.
 *
 * @param handle		The performance counter's handle.
 */
__EXPORT extern void		perf_free(perf_counter_t handle);  //释放一个计数器

/**
 * Count a performance event.
 *
 * This call only affects counters that take single events; PC_COUNT, PC_INTERVAL etc.
 *
 * @param handle		The handle returned from perf_alloc.
 */
__EXPORT extern void		perf_count(perf_counter_t handle);  //计算机性能时间

/**
 * Begin a performance event.
 *
 * This call applies to counters that operate over ranges of time; PC_ELAPSED etc.
 *
 * @param handle		The handle returned from perf_alloc.
 */
__EXPORT extern void		perf_begin(perf_counter_t handle);  //开始一个性能事件

/**
 * End a performance event.
 *
 * This call applies to counters that operate over ranges of time; PC_ELAPSED etc.
 * If a call is made without a corresponding perf_begin call, or if perf_cancel
 * has been called subsequently, no change is made to the counter.
 *
 * @param handle		The handle returned from perf_alloc.
 */
__EXPORT extern void		perf_end(perf_counter_t handle);  //结束一个性能事件

/**
 * Register a measurement
 *
 * This call applies to counters that operate over ranges of time; PC_ELAPSED etc.
 * If a call is made without a corresponding perf_begin call. It sets the
 * value provided as argument as a new measurement.
 *
 * @param handle		The handle returned from perf_alloc.
 * @param elapsed		The time elapsed. Negative values lead to incrementing the overrun counter.
 */
__EXPORT extern void		perf_set(perf_counter_t handle, int64_t elapsed);  //注册一个测量

/**
 * Cancel a performance event.
 *
 * This call applies to counters that operate over ranges of time; PC_ELAPSED etc.
 * It reverts the effect of a previous perf_begin.
 *
 * @param handle		The handle returned from perf_alloc.
 */
__EXPORT extern void		perf_cancel(perf_counter_t handle);  //取消一个性能测量事件

/**
 * Reset a performance counter.
 *
 * This call resets performance counter to initial state
 *
 * @param handle		The handle returned from perf_alloc.
 */
__EXPORT extern void		perf_reset(perf_counter_t handle);  //重启一个测量时间

/**
 * Print one performance counter to stdout
 *
 * @param handle		The counter to print.
 */
__EXPORT extern void		perf_print_counter(perf_counter_t handle);  //打印一个性能测量时间

/**
 * Print one performance counter to a fd.
 *
 * @param fd			File descriptor to print to - e.g. 0 for stdout
 * @param handle		The counter to print.
 */
__EXPORT extern void		perf_print_counter_fd(int fd, perf_counter_t handle); //答应一个性能计数到一个文件

/**
 * Print all of the performance counters.
 *
 * @param fd			File descriptor to print to - e.g. 0 for stdout
 */
__EXPORT extern void		perf_print_all(int fd);  //打印所有的性能计数器

/**
 * Print hrt latency counters.
 *
 * @param fd			File descriptor to print to - e.g. 0 for stdout
 */
__EXPORT extern void		perf_print_latency(int fd);  //打印高精度计数器延迟

/**
 * Reset all of the performance counters.
 */
__EXPORT extern void		perf_reset_all(void); //复位所有的性能计数器

/**
 * Return current event_count
 *
 * @param handle		The counter returned from perf_alloc.
 * @return			event_count
 */
__EXPORT extern uint64_t	perf_event_count(perf_counter_t handle);  //返回当前事件计数

__END_DECLS

#endif





