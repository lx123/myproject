px4_add_module(
	MODULE drivers__l3gd20
	MAIN l3gd20
	STACK 1200
	COMPILE_FLAGS
		-Weffc++
		-Os
	SRCS
		l3gd20.cpp
	DEPENDS
		platforms__common
	)
	
	