# this includes the generated topics directory
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	)
	
	set(SRCS
			objects_common.cpp
			uORBUtils.cpp
			uORB.cpp
			uORBMain.cpp
			Publication.cpp
			Subscription.cpp
			uORBDevices_nuttx.cpp
			uORBTest_UnitTest.cpp
			uORBManager_nuttx.cpp
	)
	
	px4_add_module(
		MODULE modules__uORB
		MAIN uorb
		STACK 2048
		COMPILE_FLAGS
			-Os
		SRCS ${SRCS}
		DEPENDS
			platforms__common
	)
	