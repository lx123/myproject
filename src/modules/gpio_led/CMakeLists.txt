px4_add_module(
	MODULE modules__gpio_led
	MAIN gpio_led
	COMPILE_FLAGS
		-Os
	SRCS
		gpio_led.c
	DEPENDS
		platforms__common
	)


