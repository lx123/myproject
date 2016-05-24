set(srcs
	ms5611_spi.cpp
	ms5611_i2c.cpp
	)

	list(APPEND srcs
		ms5611_nuttx.cpp
		)

px4_add_module(
	MODULE drivers__ms5611
	MAIN ms5611
	COMPILE_FLAGS
		-Os
	SRCS ${srcs}
	DEPENDS
		platforms__common
	)