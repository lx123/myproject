px4_add_module(
	MODULE drivers__boards__fmu
	COMPILE_FLAGS
		-Os
	SRCS
		fmu_init.c
		fmu_led.c
		fmu_usb.c
		fmu_spi.c
	DEPENDS
		platforms__common
	)
