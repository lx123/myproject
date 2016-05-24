px4_add_module(
	MODULE drivers__led
	COMPILE_FLAGS
		-Os
	SRCS
		led.cpp
	DEPENDS
		platforms__common
	)



