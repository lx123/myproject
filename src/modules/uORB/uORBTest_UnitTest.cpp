
#include "uORBTest_UnitTest.hpp"
#include "uORBCommon.hpp"
#include <nuttx/config.h>
#include <px4_time.h>
#include <stdio.h>

uORBTest::UnitTest &uORBTest::UnitTest::instance()
{
	static uORBTest::UnitTest t;
	return t;
}

int uORBTest::UnitTest::pubsublatency_main(void)
{
	/* poll on test topic and output latency */
	float latency_integral = 0.0f;

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[3];

	int test_multi_sub = orb_subscribe_multi(ORB_ID(orb_test), 0);
	int test_multi_sub_medium = orb_subscribe_multi(ORB_ID(orb_test_medium), 0);
	int test_multi_sub_large = orb_subscribe_multi(ORB_ID(orb_test_large), 0);

	struct orb_test_large t;

	/* clear all ready flags */
	orb_copy(ORB_ID(orb_test), test_multi_sub, &t);
	orb_copy(ORB_ID(orb_test_medium), test_multi_sub_medium, &t);
	orb_copy(ORB_ID(orb_test_large), test_multi_sub_large, &t);

	fds[0].fd = test_multi_sub;
	fds[0].events = POLLIN;
	fds[1].fd = test_multi_sub_medium;
	fds[1].events = POLLIN;
	fds[2].fd = test_multi_sub_large;
	fds[2].events = POLLIN;

	const unsigned maxruns = 1000;
	unsigned timingsgroup = 0;

	unsigned *timings = new unsigned[maxruns];

	for (unsigned i = 0; i < maxruns; i++) {
		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test), test_multi_sub, &t);
			timingsgroup = 0;

		} else if (fds[1].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test_medium), test_multi_sub_medium, &t);
			timingsgroup = 1;

		} else if (fds[2].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test_large), test_multi_sub_large, &t);
			timingsgroup = 2;
		}

		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		hrt_abstime elt = hrt_elapsed_time(&t.time);
		latency_integral += elt;
		timings[i] = elt;
	}

	orb_unsubscribe(test_multi_sub);
	orb_unsubscribe(test_multi_sub_medium);
	orb_unsubscribe(test_multi_sub_large);

	if (pubsubtest_print) {
		char fname[32];
		sprintf(fname, PX4_ROOTFSDIR"/fs/microsd/timings%u.txt", timingsgroup);
		FILE *f = fopen(fname, "w");

		if (f == NULL) {
			warnx("Error opening file!\n");
			return uORB::ERROR;
		}

		for (unsigned i = 0; i < maxruns; i++) {
			fprintf(f, "%u\n", timings[i]);
		}

		fclose(f);
	}

	delete[] timings;

	warnx("mean: %8.4f", static_cast<double>(latency_integral / maxruns));

	pubsubtest_passed = true;

	if (static_cast<float>(latency_integral / maxruns) > 30.0f) {
		pubsubtest_res = uORB::ERROR;

	} else {
		pubsubtest_res = PX4_OK;
	}

	return pubsubtest_res;
}

int uORBTest::UnitTest::test()   //测试
{
    int ret = test_single();  //单个会话测试

    if (ret != OK) {  //测试失败
		return ret;
	}

    ret = test_multi();  //多个会话测试

	if (ret != OK) {
		return ret;
	}

    ret = test_multi_reversed();  //测试多个主题颠倒

	if (ret != OK) {
		return ret;
	}

	return OK;
}


int uORBTest::UnitTest::info()
{
	return OK;
}

int uORBTest::UnitTest::test_single()
{
    test_note("try single-topic support");  //尝试单个会话支持

    struct orb_test t, u;  //定义测试结构体
	int sfd;
	orb_advert_t ptopic;
	bool updated;

	t.val = 0;
    ptopic = orb_advertise(ORB_ID(orb_test), &t);  //发布主题，第一个为主题结构体指针，第二个为数据

    if (ptopic == nullptr) {  //会话创建失败
		return test_fail("advertise failed: %d", errno);
	}

    test_note("publish handle 0x%08x", ptopic); //测试节点
    sfd = orb_subscribe(ORB_ID(orb_test)); //订阅主题，获得文件句柄

    if (sfd < 0) { //订阅失败
		return test_fail("subscribe failed: %d", errno);
	}

    test_note("subscribe fd %d", sfd); //订阅节点
	u.val = 1;

    if (PX4_OK != orb_copy(ORB_ID(orb_test), sfd, &u)) { //获取数据，orb id号/文件句柄/数据缓冲
		return test_fail("copy(1) failed: %d", errno);
	}

    if (u.val != t.val) { //数据获取失败
		return test_fail("copy(1) mismatch: %d expected %d", u.val, t.val);
	}

    if (PX4_OK != orb_check(sfd, &updated)) { //检查会话是否更新
		return test_fail("check(1) failed");
	}

    if (updated) { //如果更新了说明有问题
		return test_fail("spurious updated flag");
	}

	t.val = 2;
    test_note("try publish"); //测试发布

    if (PX4_OK != orb_publish(ORB_ID(orb_test), ptopic, &t)) { //测试发布
		return test_fail("publish failed");
	}

    if (PX4_OK != orb_check(sfd, &updated)) { //检查更新
		return test_fail("check(2) failed");
	}

    if (!updated) { //如果更新失败
		return test_fail("missing updated flag");
	}

    if (PX4_OK != orb_copy(ORB_ID(orb_test), sfd, &u)) { //获得数据
		return test_fail("copy(2) failed: %d", errno);
	}

    if (u.val != t.val) {  //获取数据有误
		return test_fail("copy(2) mismatch: %d expected %d", u.val, t.val);
	}

    orb_unsubscribe(sfd); //注销订阅

    return test_note("PASS single-topic test"); //通过单会话测试
}

int uORBTest::UnitTest::test_multi()
{
	/* this routine tests the multi-topic support */
    test_note("try multi-topic support");  //多个主题支持

	struct orb_test t, u;
	t.val = 0;
	int instance0;
    orb_advert_t pfd0 = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance0, ORB_PRIO_MAX); //结构体指针/数据//null/最高优先级

	test_note("advertised");

	int instance1;
    orb_advert_t pfd1 = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance1, ORB_PRIO_MIN); //第二个会话，最低优先级

    if (instance0 != 0) {  //第一个标志如果不为0，则为错误
		return test_fail("mult. id0: %d", instance0);
	}

    if (instance1 != 1) { //第二个话题如果不为1，则为错误
		return test_fail("mult. id1: %d", instance1);
	}

	t.val = 103;

    if (PX4_OK != orb_publish(ORB_ID(orb_multitest), pfd0, &t)) { //第一个话题发布数据
		return test_fail("mult. pub0 fail");
	}

	test_note("published");

	t.val = 203;

    if (PX4_OK != orb_publish(ORB_ID(orb_multitest), pfd1, &t)) { //第二个数据发布数据
		return test_fail("mult. pub1 fail");
	}

	/* subscribe to both topics and ensure valid data is received */
    int sfd0 = orb_subscribe_multi(ORB_ID(orb_multitest), 0);  //订阅两个主题，保证数据有效接收，后面的数据表示话题编号

    if (PX4_OK != orb_copy(ORB_ID(orb_multitest), sfd0, &u)) { //从第一个主题中获得数据
		return test_fail("sub #0 copy failed: %d", errno);
	}

	if (u.val != 103) {
		return test_fail("sub #0 val. mismatch: %d", u.val);
	}

    int sfd1 = orb_subscribe_multi(ORB_ID(orb_multitest), 1); //订阅第二个话题

    if (PX4_OK != orb_copy(ORB_ID(orb_multitest), sfd1, &u)) { //读取第二个话题数据
		return test_fail("sub #1 copy failed: %d", errno);
	}

    if (u.val != 203) { //读取数据有误
		return test_fail("sub #1 val. mismatch: %d", u.val);
	}

	/* test priorities */
    int prio;  //测试优先级

    if (PX4_OK != orb_priority(sfd0, &prio)) {//读取第一个话题优先级
		return test_fail("prio #0");
	}

    if (prio != ORB_PRIO_MAX) {  //优先级有误
		return test_fail("prio: %d", prio);
	}

    if (PX4_OK != orb_priority(sfd1, &prio)) {  //读取第二个话题优先级
		return test_fail("prio #1");
	}

    if (prio != ORB_PRIO_MIN) {  //优先级有错误
		return test_fail("prio: %d", prio);
	}

    if (PX4_OK != latency_test<struct orb_test>(ORB_ID(orb_test), false)) { //延迟测试
		return test_fail("latency test failed");
	}

	return test_note("PASS multi-topic test");
}

int uORBTest::UnitTest::test_multi_reversed()
{
    test_note("try multi-topic support subscribing before publishing"); //尝试多个主题支持的订阅，在发布之前

	/* For these tests 0 and 1 instances are taken from before, therefore continue with 2 and 3. */

	/* Subscribe first and advertise afterwards. */
    int sfd2 = orb_subscribe_multi(ORB_ID(orb_multitest), 2); //订阅多个主题，不存在的话题2

    if (sfd2 < 0) { //如果返回空，则表示失败！意思表示没有发布也能订阅数据
		return test_fail("sub. id2: ret: %d", sfd2);
	}

	struct orb_test t, u;

	t.val = 0;

	int instance2;

    orb_advert_t pfd2 = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance2, ORB_PRIO_MAX); //发布话题2

	int instance3;

    orb_advert_t pfd3 = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance3, ORB_PRIO_MIN); //发布话题3

    test_note("advertised");  //测试发布

    if (instance2 != 2) {  //发布错误
		return test_fail("mult. id2: %d", instance2);
	}

    if (instance3 != 3) {  //发布错误
		return test_fail("mult. id3: %d", instance3);
	}

	t.val = 204;

    if (PX4_OK != orb_publish(ORB_ID(orb_multitest), pfd2, &t)) { //话题2发布
		return test_fail("mult. pub0 fail");
	}


	t.val = 304;

    if (PX4_OK != orb_publish(ORB_ID(orb_multitest), pfd3, &t)) {  //话题3发布
		return test_fail("mult. pub1 fail");
	}

	test_note("published");

    if (PX4_OK != orb_copy(ORB_ID(orb_multitest), sfd2, &u)) {  //获得话题2的数据
		return test_fail("sub #2 copy failed: %d", errno);
	}

    if (u.val != 204) {  //话题2的数据不想等
		return test_fail("sub #3 val. mismatch: %d", u.val);
	}

    int sfd3 = orb_subscribe_multi(ORB_ID(orb_multitest), 3);  //订阅话题3的数据

    if (PX4_OK != orb_copy(ORB_ID(orb_multitest), sfd3, &u)) { //获得话题3的数据
		return test_fail("sub #3 copy failed: %d", errno);
	}

    if (u.val != 304) { //话题3的数据不相等
		return test_fail("sub #3 val. mismatch: %d", u.val);
	}

    return test_note("PASS multi-topic reversed"); //通过多个主题的反复
}

int uORBTest::UnitTest::test_fail(const char *fmt, ...)
{
	va_list ap;

	fprintf(stderr, "uORB FAIL: ");
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, "\n");
	fflush(stderr);
	return uORB::ERROR;
}

int uORBTest::UnitTest::test_note(const char *fmt, ...)
{
	va_list ap;

	fprintf(stderr, "uORB note: ");
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, "\n");
	fflush(stderr);
	return OK;
}

int uORBTest::UnitTest::pubsubtest_threadEntry(char *const argv[])
{
	uORBTest::UnitTest &t = uORBTest::UnitTest::instance();
	return t.pubsublatency_main();
}


