
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_custom_command(OUTPUT mixer_multirotor.generated.h
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/multi_tables.py
	> mixer_multirotor.generated.h)

add_custom_target(mixer_gen
	DEPENDS mixer_multirotor.generated.h multi_tables.py)

px4_add_module(
	MODULE modules__systemlib__mixer
	SRCS
		mixer.cpp
		mixer_group.cpp
		mixer_multirotor.cpp
		mixer_simple.cpp
		mixer_load.c
		mixer_multirotor.generated.h
	DEPENDS
		platforms__common
		mixer_gen
	)