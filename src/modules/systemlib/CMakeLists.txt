# for generated files
include_directories(${CMAKE_BINARY_DIR}/src/modules/param)

set(SRCS
	perf_counter.c
	conversions.c
	#cpuload.c
	#pid/pid.c
	#airspeed.c
	#mavlink_log.c
	#rc_check.c
	#otp.c
	#board_serial.c
	#pwm_limit/pwm_limit.c
	#mcu_version.c
	#bson/tinybson.c
	#circuit_breaker.cpp
	err.c
	#printload.c
	#param/param.c
	#up_cxxinitialize.c 
	#hx_stream.c
	)

px4_add_module(
	MODULE modules__systemlib
	COMPILE_FLAGS
		-Wno-sign-compare
		-Os
	SRCS ${SRCS}
	DEPENDS
		platforms__common
		#modules__param
	)
