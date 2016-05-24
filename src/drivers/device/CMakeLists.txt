set(SRCS)

list(APPEND SRCS
	ringbuffer.cpp
	integrator.cpp
	device_nuttx.cpp
	cdev.cpp
	i2c_nuttx.cpp
	pio.cpp
	spi.cpp
)

px4_add_module(
	MODULE drivers__device
	SRCS ${SRCS}
	DEPENDS
		platforms__common
	)
