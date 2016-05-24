/**
 * @file px4_tasks.h
 * Preserve existing task API call signature with OS abstraction
 */


#pragma once

#include <stdbool.h>
#include <sched.h>

typedef int px4_task_t;

#include <sys/prctl.h>
#define px4_prctl prctl

#define SCHED_DEFAULT  SCHED_FIFO

#define px4_task_exit(x) _exit(x)


typedef int (*px4_main_t)(int argc, char *argv[]);

__BEGIN_DECLS

/** Reboots the board */
__EXPORT void px4_systemreset(bool to_bootloader) noreturn_function;

/** Starts a task and performs any specific accounting, scheduler setup, etc. */
__EXPORT px4_task_t px4_task_spawn_cmd(const char *name,
				       int priority,
				       int scheduler,
				       int stack_size,
				       px4_main_t entry,
				       char *const argv[]);

/** Deletes a task - does not do resource cleanup **/
__EXPORT int px4_task_delete(px4_task_t pid);

/** Send a signal to a task **/
__EXPORT int px4_task_kill(px4_task_t pid, int sig);

/** Exit current task with return value **/
__EXPORT void px4_task_exit(int ret);

/** Show a list of running tasks **/
__EXPORT void px4_show_tasks(void);

/** See if a task is running **/
__EXPORT bool px4_task_is_running(const char *taskname);


__END_DECLS



