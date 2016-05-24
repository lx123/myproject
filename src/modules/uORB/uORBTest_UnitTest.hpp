
#ifndef _uORBTest_UnitTest_hpp_
#define _uORBTest_UnitTest_hpp_
#include "uORBCommon.hpp"
#include "uORB.h"
#include <px4_time.h>
#include <px4_tasks.h>

struct orb_test {  //元数据结构体
	int val;
	hrt_abstime time;
};
ORB_DEFINE(orb_test, struct orb_test);  //定义uORB结构体
ORB_DEFINE(orb_multitest, struct orb_test); //多个主题测试

struct orb_test_medium {
	int val;
	hrt_abstime time;
	char junk[64];
};
ORB_DEFINE(orb_test_medium, struct orb_test_medium);

struct orb_test_large {
	int val;
	hrt_abstime time;
	char junk[512];
};
ORB_DEFINE(orb_test_large, struct orb_test_large);


namespace uORBTest
{
class UnitTest;
}

class uORBTest::UnitTest
{
public:

	// Singleton pattern
    static uORBTest::UnitTest &instance(); //定义个返回类的函数
	~UnitTest() {}
	int test();
	template<typename S> int latency_test(orb_id_t T, bool print);
	int info();

private:
	UnitTest() : pubsubtest_passed(false), pubsubtest_print(false) {}

	// Disallow copy
	UnitTest(const uORBTest::UnitTest &) {};
	static int pubsubtest_threadEntry(char *const argv[]);
	int pubsublatency_main(void);
	//
	bool pubsubtest_passed;
	bool pubsubtest_print;
	int pubsubtest_res = OK;

	int test_single();
	int test_multi();
	int test_multi_reversed();

	int test_fail(const char *fmt, ...);
	int test_note(const char *fmt, ...);
};

template<typename S>
int uORBTest::UnitTest::latency_test(orb_id_t T, bool print)
{
	test_note("---------------- LATENCY TEST ------------------");
	S t;
	t.val = 308;
	t.time = hrt_absolute_time();

	orb_advert_t pfd0 = orb_advertise(T, &t);

	char *const args[1] = { NULL };

	pubsubtest_print = print;
	pubsubtest_passed = false;

	/* test pub / sub latency */

	// Can't pass a pointer in args, must be a null terminated
	// array of strings because the strings are copied to
	// prevent access if the caller data goes out of scope
	int pubsub_task = px4_task_spawn_cmd("uorb_latency",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_MAX - 5,
					     1500,
					     (px4_main_t)&uORBTest::UnitTest::pubsubtest_threadEntry,
					     args);

	/* give the test task some data */
	while (!pubsubtest_passed) {
		t.val = 308;
		t.time = hrt_absolute_time();

		if (PX4_OK != orb_publish(T, pfd0, &t)) {
			return test_fail("mult. pub0 timing fail");
		}

		/* simulate >800 Hz system operation */
		usleep(1000);
	}

	if (pubsub_task < 0) {
		return test_fail("failed launching task");
	}

	return pubsubtest_res;
}

#endif // _uORBTest_UnitTest_hpp_


