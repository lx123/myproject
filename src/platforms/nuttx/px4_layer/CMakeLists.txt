px4_add_module(
	MODULE platforms__nuttx__px4_layer
	COMPILE_FLAGS
		-Os
	SRCS
		px4_nuttx_tasks.c
		../../posix/px4_layer/px4_log.c
	DEPENDS
		platforms__common
	)
	
	