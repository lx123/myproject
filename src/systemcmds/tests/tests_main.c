#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include "stm32.h"

#include <arch/board/board.h>
//#include <systemlib/perf_counter.h>

// Not using Eigen at the moment
#define TESTS_EIGEN_DISABLE

#include "tests.h"

static int test_help(int argc, char *argv[]);
//static int test_all(int argc, char *argv[]);
//static int test_perf(int argc, char *argv[]);
//static int test_jig(int argc, char *argv[]);

const struct {//结构体
	const char 	*name;//名字
	int	(* fn)(int argc, char *argv[]);//调用的函数
	unsigned	options;//选项
#define OPT_NOHELP	(1<<0)
#define OPT_NOALLTEST	(1<<1)
#define OPT_NOJIGTEST	(1<<2)
} tests[] = {
	{"led",			test_led,	0},
//	{"int",			test_int,	0},
//	{"float",		test_float,	0},
//	{"sensors",		test_sensors,	0},
//	{"gpio",		test_gpio,	OPT_NOJIGTEST | OPT_NOALLTEST},
//	{"hrt",			test_hrt,	OPT_NOJIGTEST | OPT_NOALLTEST},
//	{"ppm",			test_ppm,	OPT_NOJIGTEST | OPT_NOALLTEST},
//	{"servo",		test_servo,	OPT_NOJIGTEST | OPT_NOALLTEST},
//	{"ppm_loopback",	test_ppm_loopback,	OPT_NOALLTEST},
//	{"adc",			test_adc,	OPT_NOJIGTEST},
//	{"jig_voltages",	test_jig_voltages,	OPT_NOALLTEST},
//	{"uart_loopback",	test_uart_loopback,	OPT_NOJIGTEST | OPT_NOALLTEST},
//	{"uart_baudchange",	test_uart_baudchange,	OPT_NOJIGTEST | OPT_NOALLTEST},
//	{"uart_send",		test_uart_send,	OPT_NOJIGTEST | OPT_NOALLTEST},
//	{"uart_console",	test_uart_console,	OPT_NOJIGTEST | OPT_NOALLTEST},
//	{"hott_telemetry",	test_hott_telemetry,	OPT_NOJIGTEST | OPT_NOALLTEST},
//	{"tone",		test_tone,	0},
//	{"sleep",		test_sleep,	OPT_NOJIGTEST},
//	{"time",		test_time,	OPT_NOJIGTEST},
//	{"perf",		test_perf,	OPT_NOJIGTEST},
//	{"all",			test_all,	OPT_NOALLTEST | OPT_NOJIGTEST},
//	{"jig",			test_jig,	OPT_NOJIGTEST | OPT_NOALLTEST},
//	{"param",		test_param,	0},
//	{"bson",		test_bson,	0},
//	{"file",		test_file,	OPT_NOJIGTEST | OPT_NOALLTEST},
//	{"file2",		test_file2,	OPT_NOJIGTEST},
//	{"mixer",		test_mixer,	OPT_NOJIGTEST | OPT_NOALLTEST},
//	{"rc",			test_rc,	OPT_NOJIGTEST | OPT_NOALLTEST},
//	{"conv",		test_conv,	OPT_NOJIGTEST | OPT_NOALLTEST},
//	{"mount",		test_mount,	OPT_NOJIGTEST | OPT_NOALLTEST},
#ifndef TESTS_MATHLIB_DISABLE
//	{"mathlib",		test_mathlib,	0},
#endif
#ifndef TESTS_EIGEN_DISABLE
	{"eigen",		test_eigen,	OPT_NOJIGTEST},
#endif
	{"help",		test_help,	OPT_NOALLTEST | OPT_NOHELP | OPT_NOJIGTEST},
	{NULL,			NULL, 		0}
};

#define NTESTS (sizeof(tests) / sizeof(tests[0]))

static int
test_help(int argc, char *argv[])
{
	unsigned	i;

	printf("Available tests:\n");  //打印支持的测试

	for (i = 0; tests[i].name; i++) {
		printf("  %s\n", tests[i].name);
	}

	return 0;
}

//static int
//test_all(int argc, char *argv[])
//{
//	unsigned	i;
//	char		*args[2] = {"all", NULL};
//	unsigned int failcount = 0;
//	unsigned int testcount = 0;
//	bool		passed[NTESTS];
//
//	printf("\nRunning all tests...\n\n");
//
//	for (i = 0; tests[i].name; i++) {
//		/* Only run tests that are not excluded */
//		if (!(tests[i].options & OPT_NOALLTEST)) {
//			printf("  [%s] \t\t\tSTARTING TEST\n", tests[i].name);
//			fflush(stdout);
//
//			/* Execute test */
//			if (tests[i].fn(1, args) != 0) {
//				fprintf(stderr, "  [%s] \t\t\tFAIL\n", tests[i].name);
//				fflush(stderr);
//				failcount++;
//				passed[i] = false;
//
//			} else {
//				printf("  [%s] \t\t\tPASS\n", tests[i].name);
//				fflush(stdout);
//				passed[i] = true;
//			}
//
//			testcount++;
//		}
//	}
//
//	/* Print summary */
//	printf("\n");
//	int j;
//
//	for (j = 0; j < 40; j++) {
//		printf("-");
//	}
//
//	printf("\n\n");
//
//	printf("     T E S T    S U M M A R Y\n\n");
//
//	if (failcount == 0) {
//		printf("  ______     __         __            ______     __  __    \n");
//		printf(" /\\  __ \\   /\\ \\       /\\ \\          /\\  __ \\   /\\ \\/ /    \n");
//		printf(" \\ \\  __ \\  \\ \\ \\____  \\ \\ \\____     \\ \\ \\/\\ \\  \\ \\  _\"-.  \n");
//		printf("  \\ \\_\\ \\_\\  \\ \\_____\\  \\ \\_____\\     \\ \\_____\\  \\ \\_\\ \\_\\ \n");
//		printf("   \\/_/\\/_/   \\/_____/   \\/_____/      \\/_____/   \\/_/\\/_/ \n");
//		printf("\n");
//		printf(" All tests passed (%d of %d)\n", testcount, testcount);
//
//	} else {
//		printf("  ______   ______     __     __ \n");
//		printf(" /\\  ___\\ /\\  __ \\   /\\ \\   /\\ \\    \n");
//		printf(" \\ \\  __\\ \\ \\  __ \\  \\ \\ \\  \\ \\ \\__\n");
//		printf("  \\ \\_\\    \\ \\_\\ \\_\\  \\ \\_\\  \\ \\_____\\ \n");
//		printf("   \\/_/     \\/_/\\/_/   \\/_/   \\/_____/ \n");
//		printf("\n");
//		printf(" Some tests failed (%d of %d)\n", failcount, testcount);
//	}
//
//	printf("\n");
//
//	/* Print failed tests */
//	if (failcount > 0) { printf(" Failed tests:\n\n"); }
//
//	unsigned int k;
//
//	for (k = 0; k < i; k++) {
//		if (!passed[k] && !(tests[k].options & OPT_NOALLTEST)) {
//			printf(" [%s] to obtain details, please re-run with\n\t nsh> tests %s\n\n", tests[k].name, tests[k].name);
//		}
//	}
//
//	fflush(stdout);
//
//	return (failcount > 0);
//}


//static int
//test_perf(int argc, char *argv[])
//{
//	perf_counter_t	cc, ec;
//
//	cc = perf_alloc(PC_COUNT, "test_count");
//	ec = perf_alloc(PC_ELAPSED, "test_elapsed");
//
//	if ((cc == NULL) || (ec == NULL)) {
//		printf("perf: counter alloc failed\n");
//		return 1;
//	}
//
//	perf_begin(ec);
//	perf_count(cc);
//	perf_count(cc);
//	perf_count(cc);
//	perf_count(cc);
//	printf("perf: expect count of 4\n");
//	perf_print_counter(cc);
//	perf_end(ec);
//	printf("perf: expect count of 1\n");
//	perf_print_counter(ec);
//	printf("perf: expect at least two counters\n");
//	perf_print_all(0);
//
//	perf_free(cc);
//	perf_free(ec);
//
//	return OK;
//}

//int test_jig(int argc, char *argv[])
//{
//	unsigned	i;
//	char		*args[2] = {"jig", NULL};
//	unsigned int failcount = 0;
//	unsigned int testcount = 0;
//	bool		passed[NTESTS];
//
//	printf("\nRunning all tests...\n\n");
//
//	for (i = 0; tests[i].name; i++) {
//		/* Only run tests that are not excluded */
//		if (!(tests[i].options & OPT_NOJIGTEST)) {
//			printf("  [%s] \t\t\tSTARTING TEST\n", tests[i].name);
//			fflush(stdout);
//
//			/* Execute test */
//			if (tests[i].fn(1, args) != 0) {
//				fprintf(stderr, "  [%s] \t\t\tFAIL\n", tests[i].name);
//				fflush(stderr);
//				failcount++;
//				passed[i] = false;
//
//			} else {
//				printf("  [%s] \t\t\tPASS\n", tests[i].name);
//				fflush(stdout);
//				passed[i] = true;
//			}
//
//			testcount++;
//		}
//	}
//
//	/* Print summary */
//	printf("\n");
//	int j;
//
//	for (j = 0; j < 40; j++) {
//		printf("-");
//	}
//
//	printf("\n\n");
//
//	printf("     T E S T    S U M M A R Y\n\n");
//
//	if (failcount == 0) {
//		printf("  ______     __         __            ______     __  __    \n");
//		printf(" /\\  __ \\   /\\ \\       /\\ \\          /\\  __ \\   /\\ \\/ /    \n");
//		printf(" \\ \\  __ \\  \\ \\ \\____  \\ \\ \\____     \\ \\ \\/\\ \\  \\ \\  _\"-.  \n");
//		printf("  \\ \\_\\ \\_\\  \\ \\_____\\  \\ \\_____\\     \\ \\_____\\  \\ \\_\\ \\_\\ \n");
//		printf("   \\/_/\\/_/   \\/_____/   \\/_____/      \\/_____/   \\/_/\\/_/ \n");
//		printf("\n");
//		printf(" All tests passed (%d of %d)\n", testcount, testcount);
//
//	} else {
//		printf("  ______   ______     __     __ \n");
//		printf(" /\\  ___\\ /\\  __ \\   /\\ \\   /\\ \\    \n");
//		printf(" \\ \\  __\\ \\ \\  __ \\  \\ \\ \\  \\ \\ \\__\n");
//		printf("  \\ \\_\\    \\ \\_\\ \\_\\  \\ \\_\\  \\ \\_____\\ \n");
//		printf("   \\/_/     \\/_/\\/_/   \\/_/   \\/_____/ \n");
//		printf("\n");
//		printf(" Some tests failed (%d of %d)\n", failcount, testcount);
//	}
//
//	printf("\n");
//
//	/* Print failed tests */
//	if (failcount > 0) { printf(" Failed tests:\n\n"); }
//
//	unsigned int k;
//
//	for (k = 0; k < i; k++) {
//		if (!passed[i] && !(tests[k].options & OPT_NOJIGTEST)) {
//			printf(" [%s] to obtain details, please re-run with\n\t nsh> tests %s\n\n", tests[k].name, tests[k].name);
//		}
//	}
//
//	fflush(stdout);
//
//	return 0;
//}

__EXPORT int tests_main(int argc, char *argv[]);

/**
 * Executes system tests. 执行系统测试
 */
int tests_main(int argc, char *argv[])
{
	unsigned	i;

	if (argc < 2) {
		printf("tests: missing test name - 'tests help' for a list of tests\n");
		return 1;
	}

	for (i = 0; tests[i].name; i++) {
		if (!strcmp(tests[i].name, argv[1])) { //如果找到对应模块
			return tests[i].fn(argc - 1, argv + 1);
		}
	}

	printf("tests: no test called '%s' - 'tests help' for a list of tests\n", argv[1]);
	return ERROR;
}










