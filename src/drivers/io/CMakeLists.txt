px4_add_module(
	MODULE drivers__px4io
	MAIN px4io
	STACK 1800
	COMPILE_FLAGS
		-Os
	SRCS
		px4io.cpp
		px4io_uploader.cpp
		px4io_serial.cpp
		px4io_i2c.cpp
		px4io_params.c
	DEPENDS
		platforms__common
	)
	
	