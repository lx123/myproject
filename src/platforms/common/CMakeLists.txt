set(depends
#	msg_gen
	prebuild_targets
#	git_mavlink
#	git_uavcan
#	git_matrix
	)
	
px4_add_module(
	MODULE platforms__common
	SRCS
		px4_getopt.c
	DEPENDS
		${depends}
	)