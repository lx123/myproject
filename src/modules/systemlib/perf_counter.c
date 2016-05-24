/**
 * @file perf_counter.c
 *
 * @brief Performance measuring tools.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/queue.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include "perf_counter.h"

#define ddeclare(...) __VA_ARGS__

/**
 * Header common to all counters. 所有计数器的共同头部
 */
struct perf_ctr_header {
	sq_entry_t		link;	/**< list linkage */  //列表连接
	enum perf_counter_type	type;	/**< counter type */   //计数器类型
	const char		*name;	/**< counter name */   //计数器名称
};


/**
 * PC_EVENT counter.  性能计数事件计数器
 */
struct perf_ctr_count {
	struct perf_ctr_header	hdr;  //计数器头部
	uint64_t		event_count;  //时间计数
};

/**
 * PC_ELAPSED counter.  性能计数器消耗时间计数器
 */
struct perf_ctr_elapsed {
	struct perf_ctr_header	hdr;   //头部
	uint64_t		event_count;   //事件计数
	uint64_t		event_overruns; //过载
	uint64_t		time_start;     //开始时间
	uint64_t		time_total;    //总共时间
	uint64_t		time_least;    //最少时间
	uint64_t		time_most;    //最多时间
	float			mean;        //平均值
	float			M2;
};

/**
 * PC_INTERVAL counter.    性能计数器间隔计数器
 */
struct perf_ctr_interval {
	struct perf_ctr_header	hdr;   //头部
	uint64_t		event_count;   //事件计数
	uint64_t		time_event;     //事件时间
	uint64_t		time_first;    //第一个时间
	uint64_t		time_last;   //最后一个时间
	uint64_t		time_least;   //最少时间
	uint64_t		time_most;     //最多时间
	float			mean;         //平均时间
	float			M2;
};

/**
 * List of all known counters.
 */
static sq_queue_t	perf_counters;   //列出所有的已知计数器

//创建一个本地计数器
perf_counter_t
perf_alloc(enum perf_counter_type type, const char *name)
{
	perf_counter_t ctr = NULL;

	switch (type) {
	case PC_COUNT:
		ctr = (perf_counter_t)calloc(sizeof(struct perf_ctr_count), 1);
		break;

	case PC_ELAPSED:
		ctr = (perf_counter_t)calloc(sizeof(struct perf_ctr_elapsed), 1);
		break;

	case PC_INTERVAL:
		ctr = (perf_counter_t)calloc(sizeof(struct perf_ctr_interval), 1);

		break;

	default:
		break;
	}

	if (ctr != NULL) {
		ctr->type = type;
		ctr->name = name;
		sq_addfirst(&ctr->link, &perf_counters);
	}

	return ctr;  //返回创建的本地计数器
}
//获得参考计数器
perf_counter_t
perf_alloc_once(enum perf_counter_type type, const char *name)
{
	perf_counter_t handle = (perf_counter_t)sq_peek(&perf_counters);

	while (handle != NULL) {
		if (!strcmp(handle->name, name)) {
			if (type == handle->type) {
				/* they are the same counter */
				return handle;

			} else {
				/* same name but different type, assuming this is an error and not intended */
				return NULL;
			}
		}

		handle = (perf_counter_t)sq_next(&handle->link);
	}

	/* if the execution reaches here, no existing counter of that name was found */
	return perf_alloc(type, name);
}
//释放性能计数器
void
perf_free(perf_counter_t handle)
{
	if (handle == NULL) {
		return;
	}

	sq_rem(&handle->link, &perf_counters);  //在队列中移除
	free(handle);
}
//性能计数器计数
void
perf_count(perf_counter_t handle)
{
	if (handle == NULL) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		((struct perf_ctr_count *)handle)->event_count++;  //事件加1
		break;

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			hrt_abstime now = hrt_absolute_time();  //获取绝对时间

			switch (pci->event_count) {
			case 0:
				pci->time_first = now;
				break;

			case 1:
				pci->time_least = now - pci->time_last;
				pci->time_most = now - pci->time_last;
				pci->mean = pci->time_least / 1e6f;
				pci->M2 = 0;
				break;

			default: {
					hrt_abstime interval = now - pci->time_last;

					if (interval < pci->time_least) {
						pci->time_least = interval;
					}

					if (interval > pci->time_most) {
						pci->time_most = interval;
					}

					// maintain mean and variance of interval in seconds
					// Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
					float dt = interval / 1e6f;
					float delta_intvl = dt - pci->mean;
					pci->mean += delta_intvl / pci->event_count;
					pci->M2 += delta_intvl * (dt - pci->mean);
					break;
				}
			}

			pci->time_last = now;
			pci->event_count++;
			break;
		}

	default:
		break;
	}
}
//性能计数器开始
void
perf_begin(perf_counter_t handle)
{
	if (handle == NULL) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED:
		((struct perf_ctr_elapsed *)handle)->time_start = hrt_absolute_time();
		break;

	default:
		break;
	}
}
//结束一个性能事件
void
perf_end(perf_counter_t handle)
{
	if (handle == NULL) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			if (pce->time_start != 0) {
				int64_t elapsed = hrt_absolute_time() - pce->time_start;

				if (elapsed < 0) {
					pce->event_overruns++;

				} else {

					pce->event_count++;
					pce->time_total += elapsed;

					if ((pce->time_least > (uint64_t)elapsed) || (pce->time_least == 0)) {
						pce->time_least = elapsed;
					}

					if (pce->time_most < (uint64_t)elapsed) {
						pce->time_most = elapsed;
					}

					// maintain mean and variance of the elapsed time in seconds
					// Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
					float dt = elapsed / 1e6f;
					float delta_intvl = dt - pce->mean;
					pce->mean += delta_intvl / pce->event_count;
					pce->M2 += delta_intvl * (dt - pce->mean);

					pce->time_start = 0;
				}
			}
		}
		break;

	default:
		break;
	}
}

#include <systemlib/err.h>
//注册一个测量量
void
perf_set(perf_counter_t handle, int64_t elapsed)
{
	if (handle == NULL) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			if (elapsed < 0) {
				pce->event_overruns++;

			} else {

				pce->event_count++;
				pce->time_total += elapsed;

				if ((pce->time_least > (uint64_t)elapsed) || (pce->time_least == 0)) {
					pce->time_least = elapsed;
				}

				if (pce->time_most < (uint64_t)elapsed) {
					pce->time_most = elapsed;
				}

				// maintain mean and variance of the elapsed time in seconds
				// Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
				float dt = elapsed / 1e6f;
				float delta_intvl = dt - pce->mean;
				pce->mean += delta_intvl / pce->event_count;
				pce->M2 += delta_intvl * (dt - pce->mean);

				pce->time_start = 0;
			}
		}
		break;

	default:
		break;
	}
}
//取消
void
perf_cancel(perf_counter_t handle)
{
	if (handle == NULL) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			pce->time_start = 0;
		}
		break;

	default:
		break;
	}
}


//性能计数复位
void
perf_reset(perf_counter_t handle)
{
	if (handle == NULL) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		((struct perf_ctr_count *)handle)->event_count = 0;
		break;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			pce->event_count = 0;
			pce->time_start = 0;
			pce->time_total = 0;
			pce->time_least = 0;
			pce->time_most = 0;
			break;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			pci->event_count = 0;
			pci->time_event = 0;
			pci->time_first = 0;
			pci->time_last = 0;
			pci->time_least = 0;
			pci->time_most = 0;
			break;
		}
	}
}
//答应性能计数器
void
perf_print_counter(perf_counter_t handle)
{
	perf_print_counter_fd(1, handle);
}

void
perf_print_counter_fd(int fd, perf_counter_t handle)
{
	if (handle == NULL) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		dprintf(fd, "%s: %llu events\n",
			handle->name,
			(unsigned long long)((struct perf_ctr_count *)handle)->event_count);
		break;

	case PC_ELAPSED: {
			ddeclare(struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;)
			ddeclare(float rms = sqrtf(pce->M2 / (pce->event_count - 1));)
			dprintf(fd, "%s: %llu events, %llu overruns, %lluus elapsed, %lluus avg, min %lluus max %lluus %5.3fus rms\n",
				handle->name,
				(unsigned long long)pce->event_count,
				(unsigned long long)pce->event_overruns,
				(unsigned long long)pce->time_total,
				pce->event_count == 0 ? 0 : (unsigned long long)pce->time_total / pce->event_count,
				(unsigned long long)pce->time_least,
				(unsigned long long)pce->time_most,
				(double)(1e6f * rms));
			break;
		}

	case PC_INTERVAL: {
			ddeclare(struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;)
			ddeclare(float rms = sqrtf(pci->M2 / (pci->event_count - 1));)

			dprintf(fd, "%s: %llu events, %lluus avg, min %lluus max %lluus %5.3fus rms\n",
				handle->name,
				(unsigned long long)pci->event_count,
				(unsigned long long)(pci->time_last - pci->time_first) / pci->event_count,
				(unsigned long long)pci->time_least,
				(unsigned long long)pci->time_most,
				(double)(1e6f * rms));
			break;
		}

	default:
		break;
	}
}

uint64_t
perf_event_count(perf_counter_t handle)
{
	if (handle == NULL) {
		return 0;
	}

	switch (handle->type) {
	case PC_COUNT:
		return ((struct perf_ctr_count *)handle)->event_count;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			return pce->event_count;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			return pci->event_count;
		}

	default:
		break;
	}

	return 0;
}

void
perf_print_all(int fd)
{
	perf_counter_t handle = (perf_counter_t)sq_peek(&perf_counters);

	while (handle != NULL) {
		perf_print_counter_fd(fd, handle);
		handle = (perf_counter_t)sq_next(&handle->link);
	}
}

extern const uint16_t latency_bucket_count;
extern uint32_t latency_counters[];
extern const uint16_t latency_buckets[];

void
perf_print_latency(int fd)
{
	dprintf(fd, "bucket : events\n");

	for (int i = 0; i < latency_bucket_count; i++) {
		printf("  %4i : %li\n", latency_buckets[i], (long int)latency_counters[i]);
	}

	// print the overflow bucket value
	dprintf(fd, " >%4i : %i\n", latency_buckets[latency_bucket_count - 1], latency_counters[latency_bucket_count]);
}

void
perf_reset_all(void)
{
	perf_counter_t handle = (perf_counter_t)sq_peek(&perf_counters);

	while (handle != NULL) {
		perf_reset(handle);
		handle = (perf_counter_t)sq_next(&handle->link);
	}

	for (int i = 0; i <= latency_bucket_count; i++) {
		latency_counters[i] = 0;
	}
}


