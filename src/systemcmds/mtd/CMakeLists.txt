px4_add_module(
	MODULE systemcmds__mtd
	MAIN mtd
	COMPILE_FLAGS
		-Wno-error
		-Os
	SRCS
		mtd.c
	DEPENDS
		platforms__common
	)
	