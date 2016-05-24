px4_add_module(
	MODULE drivers__lsm303d
	MAIN lsm303d
	STACK 1200
	COMPILE_FLAGS
		-Weffc++
		-Os
	SRCS
		lsm303d.cpp
	DEPENDS
		platforms__common
	)
	
	
	