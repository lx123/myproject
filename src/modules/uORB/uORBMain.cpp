
#include <string.h>
#include "uORBDevices_nuttx.hpp"
#include "uORB.h"
#include "uORBCommon.hpp"

#include "uORBTest_UnitTest.hpp"

extern "C" { __EXPORT int uorb_main(int argc, char *argv[]); }  //供外部c语言调用

static uORB::DeviceMaster *g_dev = nullptr;  //定义一个主设备实例
static void usage()
{
	PX4_INFO("Usage: uorb 'start', 'test', 'latency_test' or 'status'");
}


int
uorb_main(int argc, char *argv[])  //进程入口
{
	if (argc < 2) {
		usage();
		return -EINVAL;
	}

	/*
	 * Start/load the driver.
	 *
	 * XXX it would be nice to have a wrapper for this...
	 */
	if (!strcmp(argv[1], "start")) {

        if (g_dev != nullptr) {  //设备为空
            PX4_WARN("already loaded");  //已经载入
			/* user wanted to start uorb, its already running, no error */
			return 0;
		}

		/* create the driver */
        g_dev = new uORB::DeviceMaster(uORB::PUBSUB);  //新建一个实例，主对象"obj_master"，并注册字符设备

        if (g_dev == nullptr) { //实例化失败
			PX4_ERR("driver alloc failed");
			return -ENOMEM;
		}

        if (OK != g_dev->init()) { //初始化，字符设备
			PX4_ERR("driver init failed");
			delete g_dev;
			g_dev = nullptr;
			return -EIO;
		}

		return OK;
	}

#ifndef __PX4_QURT

	/*
	 * Test the driver/device.
	 */
    if (!strcmp(argv[1], "test")) {  //测试
        uORBTest::UnitTest &t = uORBTest::UnitTest::instance();//新建测试实例，定义一个返回类的函数，返回一个类
		return t.test();
	}

	/*
	 * Test the latency.
	 */
	if (!strcmp(argv[1], "latency_test")) {

		uORBTest::UnitTest &t = uORBTest::UnitTest::instance();

		if (argc > 2 && !strcmp(argv[2], "medium")) {
			return t.latency_test<struct orb_test_medium>(ORB_ID(orb_test_medium), true);

		} else if (argc > 2 && !strcmp(argv[2], "large")) {
			return t.latency_test<struct orb_test_large>(ORB_ID(orb_test_large), true);

		} else {
			return t.latency_test<struct orb_test>(ORB_ID(orb_test), true);
		}
	}

#endif

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "status")) {
		if (g_dev != nullptr) {
			PX4_INFO("uorb is running");

		} else {
			PX4_INFO("uorb is not running");
		}
		return OK;
	}

	usage();
	return -EINVAL;
}
