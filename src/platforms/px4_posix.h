/**
 * @file px4_posix.h
 *
 * Includes POSIX-like functions for virtual character devices
 */

#pragma once

/**
 * @file px4_posix.h
 *
 * Includes POSIX-like functions for virtual character devices
 */

#pragma once

#include <px4_defines.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <semaphore.h>
#include <sys/types.h>

/* Semaphore handling */  //信号处理
__BEGIN_DECLS

typedef sem_t px4_sem_t;

#define px4_sem_init	 sem_init
#define px4_sem_wait	 sem_wait
#define px4_sem_post	 sem_post
#define px4_sem_getvalue sem_getvalue
#define px4_sem_destroy	 sem_destroy
#define px4_sem_timedwait	 sem_timedwait
__END_DECLS

#define  PX4_F_RDONLY 1
#define  PX4_F_WRONLY 2

typedef struct pollfd px4_pollfd_struct_t;

#if defined(__cplusplus)
#define _GLOBAL ::
#else
#define _GLOBAL
#endif
#define px4_open 	_GLOBAL open
#define px4_close 	_GLOBAL close
#define px4_ioctl 	_GLOBAL ioctl
#define px4_write 	_GLOBAL write
#define px4_read 	_GLOBAL read
#define px4_poll 	_GLOBAL poll
#define px4_fsync 	_GLOBAL fsync
#define px4_access 	_GLOBAL access
#define px4_getpid 	_GLOBAL getpid

__BEGIN_DECLS
extern int px4_errno;

__EXPORT void		px4_show_devices(void);
__EXPORT void		px4_show_files(void);
__EXPORT const char 	*px4_get_device_names(unsigned int *handle);

__EXPORT void		px4_show_topics(void);
__EXPORT const char 	*px4_get_topic_names(unsigned int *handle);
__END_DECLS






