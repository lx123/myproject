px4_add_module(
	MODULE drivers__mpu6000
	MAIN mpu6000
	STACK 1200
	COMPILE_FLAGS
		-Weffc++
		-Os
	SRCS
		mpu6000.cpp
	DEPENDS
		platforms__common
	)
	
	