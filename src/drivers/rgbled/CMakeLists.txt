px4_add_module(
	MODULE drivers__rgbled
	MAIN rgbled
	COMPILE_FLAGS
		-Os
	SRCS
		rgbled.cpp
	DEPENDS
		platforms__common
	)
	