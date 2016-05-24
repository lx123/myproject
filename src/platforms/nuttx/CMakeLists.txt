px4_add_module(
	MODULE platforms__nuttx
	COMPILE_FLAGS
		-Os
	SRCS
		px4_nuttx_impl.cpp
	DEPENDS
		platforms__common
	)
	
	
	
	