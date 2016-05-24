px4_add_module(
	MODULE drivers__stm32
	COMPILE_FLAGS
		-Os
	SRCS
		drv_hrt.c
		#drv_io_timer.c
		#drv_pwm_servo.c
		#drv_input_capture.c
	DEPENDS
		platforms__common
	)
	
	
	