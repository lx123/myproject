set(srcs
#	test_adc.c
#	test_bson.c
#	test_float.c
#	test_gpio.c
#	test_hott_telemetry.c
#	test_hrt.c
#	test_int.c
#	test_jig_voltages.c
	test_led.c
#	test_sensors.c
#	test_servo.c
#	test_sleep.c
#	test_uart_baudchange.c
#	test_uart_console.c
#	test_uart_loopback.c
#	test_uart_send.c
#	test_mixer.cpp
#	test_mathlib.cpp
#	test_file.c
#	test_file2.c
	tests_main.c
#	test_params.c
#	test_ppm_loopback.c
#	test_rc.c
#	test_conv.cpp
#	test_mount.c
#	test_time.c
	)


px4_add_module(
	MODULE systemcmds__tests
	MAIN tests
	STACK 8000
	COMPILE_FLAGS
		-Wframe-larger-than=6000
		-Wno-float-equal
		-O0
	SRCS ${srcs}
	DEPENDS
		platforms__common
	)
	
	
	