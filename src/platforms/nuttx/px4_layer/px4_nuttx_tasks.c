
/**
 * @file px4_nuttx_tasks.c
 * Implementation of existing task API for NuttX
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <unistd.h>
#include <float.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>

#ifndef CONFIG_ARCH_BOARD_SIM
#include <stm32_pwr.h>
#endif

#include <systemlib/systemlib.h>

// Didn't seem right to include up_internal.h, so direct extern instead.
extern void up_systemreset(void) noreturn_function;

void
px4_systemreset(bool to_bootloader)   //系统重启，跳转到bootloader，输入为跳转标志位
{
	if (to_bootloader) {  //如果标志为真
#ifndef CONFIG_ARCH_BOARD_SIM
		stm32_pwr_enablebkp(TRUE);
#endif

		/* XXX wow, this is evil - write a magic number into backup register zero */
		*(uint32_t *)0x40002850 = 0xb007b007;  //写入备份寄存器
	}

	up_systemreset();  //系统复位

	/* lock up here */
	while (true);  //锁定在此
}

int px4_task_spawn_cmd(const char *name, int scheduler, int priority, int stack_size, main_t entry, char *const argv[])  //任务创建指令
{
	int pid; //线程id号

	sched_lock();

	/* create the task */
	pid = task_create(name, priority, stack_size, entry, argv);  //创建线程

	if (pid > 0) {  //线程创建成功

		/* configure the scheduler */
		struct sched_param param;

		param.sched_priority = priority;
		sched_setscheduler(pid, scheduler, &param);

		/* XXX do any other private task accounting here before the task starts */
	}

	sched_unlock();

	return pid;
}

int px4_task_delete(int pid)  //删除任务
{
	return task_delete(pid);
}
