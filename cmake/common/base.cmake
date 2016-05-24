include(CMakeParseArguments)

#=============================================================================
#
#	px4_parse_function_args
#
#	This function simpliies usage of the cmake_parse_arguments module.
#	It is inteded to be called by other functions.
#
#	Usage:
#		px4_parse_function_args(
#			NAME <name>
#			[ OPTIONS <list> ]
#			[ ONE_VALUE <list> ]
#			[ MULTI_VALUE <list> ]
#			REQUIRED <list>
#			ARGN <ARGN>)
#
#	Input:
#		NAME		: the name of the calling function
#		OPTIONS		: boolean flags
#		ONE_VALUE	: single value variables
#		MULTI_VALUE	: multi value variables
#		REQUIRED	: required arguments
#		ARGN		: the function input arguments, typically ${ARGN}
#
#	Output:
#		The function arguments corresponding to the following are set:
#		${OPTIONS}, ${ONE_VALUE}, ${MULTI_VALUE}
#
#	Example:
#		function test()
#			px4_parse_function_args(
#				NAME TEST
#				ONE_VALUE NAME
#				MULTI_VALUE LIST
#				REQUIRED NAME LIST
#				ARGN ${ARGN})
#			message(STATUS "name: ${NAME}")
#			message(STATUS "list: ${LIST}")
#		endfunction()
#
#		test(NAME "hello" LIST a b c)
#
#		OUTPUT:
#			name: hello
#			list: a b c
#
function(px4_parse_function_args)
	cmake_parse_arguments(IN "" "NAME" "OPTIONS;ONE_VALUE;MULTI_VALUE;REQUIRED;ARGN" "${ARGN}")
	cmake_parse_arguments(OUT "${IN_OPTIONS}" "${IN_ONE_VALUE}" "${IN_MULTI_VALUE}" "${IN_ARGN}")
	if (OUT_UNPARSED_ARGUMENTS)
		message(FATAL_ERROR "${IN_NAME}: unparsed ${OUT_UNPARSED_ARGUMENTS}")
	endif()
	foreach(arg ${IN_REQUIRED})
		if (NOT OUT_${arg})
			message(FATAL_ERROR "${IN_NAME} requires argument ${arg}\nARGN: ${IN_ARGN}")
		endif()
	endforeach()
	foreach(arg ${IN_OPTIONS} ${IN_ONE_VALUE} ${IN_MULTI_VALUE})
		set(${arg} ${OUT_${arg}} PARENT_SCOPE)
	endforeach()
endfunction()

#=============================================================================
#
#	px4_add_module
#
#	This function builds a static library from a module description.
#
#	Usage:
#		px4_add_module(MODULE <string>
#			[ MAIN <string> ]
#			[ STACK <string> ]
#			[ COMPILE_FLAGS <list> ]
#			[ INCLUDES <list> ]
#			[ DEPENDS <string> ]
#			)
#
#	Input:
#		MODULE			: unique name of module
#		MAIN			: entry point, if not given, assumed to be library
#		STACK			: size of stack
#		COMPILE_FLAGS	: compile flags
#		LINK_FLAGS		: link flags
#		SRCS			: source files
#		INCLUDES		: include directories
#		DEPENDS			: targets which this module depends on
#
#	Output:
#		Static library with name matching MODULE.
#
#	Example:
#		px4_add_module(MODULE test
#			SRCS
#				file.cpp
#			STACK 1024
#			DEPENDS
#				git_nuttx
#			)
#
function(px4_add_module)

	px4_parse_function_args(
		NAME px4_add_module
		ONE_VALUE MODULE MAIN STACK PRIORITY
		MULTI_VALUE COMPILE_FLAGS LINK_FLAGS SRCS INCLUDES DEPENDS
		REQUIRED MODULE
		ARGN ${ARGN})

	add_library(${MODULE} STATIC EXCLUDE_FROM_ALL ${SRCS})

	if(${OS} STREQUAL "qurt" )
		set_property(TARGET ${MODULE} PROPERTY POSITION_INDEPENDENT_CODE TRUE)
	endif()

	if(MAIN)
		set_target_properties(${MODULE} PROPERTIES
			COMPILE_DEFINITIONS PX4_MAIN=${MAIN}_app_main)
	endif()

	if(INCLUDES)
		target_include_directories(${MODULE} ${INCLUDES})
	endif()

	if(DEPENDS)
		add_dependencies(${MODULE} ${DEPENDS})
	endif()

	# join list variables to get ready to send to compiler
	foreach(prop LINK_FLAGS COMPILE_FLAGS)
		if(${prop})
			px4_join(OUT ${prop} LIST ${${prop}} GLUE " ")
		endif()
	endforeach()

	# store module properties in target
	# COMPILE_FLAGS and LINK_FLAGS are passed to compiler/linker by cmake
	# STACK, MAIN, PRIORITY are PX4 specific
	foreach (prop COMPILE_FLAGS LINK_FLAGS STACK MAIN PRIORITY)
		if (${prop})
			set_target_properties(${MODULE} PROPERTIES ${prop} ${${prop}})
		endif()
	endforeach()

endfunction()


#=============================================================================
#
#	px4_add_common_flags
#
#	Set ths default build flags.
#
#	Usage:
#		px4_add_common_flags(
#			BOARD <in-string>
#			C_FLAGS <inout-variable>
#			CXX_FLAGS <inout-variable>
#			EXE_LINKER_FLAGS <inout-variable>
#			INCLUDE_DIRS <inout-variable>
#			LINK_DIRS <inout-variable>
#			DEFINITIONS <inout-variable>)
#
#	Input:
#		BOARD					: board
#
#	Input/Output: (appends to existing variable)
#		C_FLAGS					: c compile flags variable
#		CXX_FLAGS				: c++ compile flags variable
#		EXE_LINKER_FLAGS		: executable linker flags variable
#		INCLUDE_DIRS			: include directories
#		LINK_DIRS				: link directories
#		DEFINITIONS				: definitions
#
#	Example:
#		px4_add_common_flags(
#			BOARD px4fmu-v2
#			C_FLAGS CMAKE_C_FLAGS
#			CXX_FLAGS CMAKE_CXX_FLAGS
#			EXE_LINKER_FLAG CMAKE_EXE_LINKER_FLAGS
#			INCLUDES <list>)
#
function(px4_add_common_flags)

	set(inout_vars
		C_FLAGS CXX_FLAGS EXE_LINKER_FLAGS INCLUDE_DIRS LINK_DIRS DEFINITIONS)

	px4_parse_function_args(
		NAME px4_add_common_flags
		ONE_VALUE ${inout_vars} BOARD
		REQUIRED ${inout_vars} BOARD
		ARGN ${ARGN})

	set(warnings
		-Wall
		-Werror
		-Wextra
		-Wno-sign-compare
		#-Wshadow # very verbose due to eigen
		-Wfloat-equal
		-Wpointer-arith
		-Wmissing-declarations
		-Wno-unused-parameter
		-Werror=format-security
		-Werror=array-bounds
		-Wfatal-errors
		-Werror=unused-variable
		-Werror=reorder
		-Werror=uninitialized
		-Werror=init-self
		#-Wcast-qual  - generates spurious noreturn attribute warnings,
		#               try again later
		#-Wconversion - would be nice, but too many "risky-but-safe"
		#               conversions in the code
		#-Wcast-align - would help catch bad casts in some cases,
		#               but generates too many false positives
		)

	if (${OS} STREQUAL "nuttx")
		list(APPEND warnings -Wframe-larger-than=1024)
	endif()

	if (${CMAKE_C_COMPILER_ID} MATCHES ".*Clang.*")
		# QuRT 6.4.X compiler identifies as Clang but does not support this option
		if (NOT ${OS} STREQUAL "qurt")
			list(APPEND warnings
				-Wno-unused-const-variable
				-Wno-varargs
			)
		endif()
	else()
		list(APPEND warnings
			-Werror=unused-but-set-variable
			-Wformat=1
			#-Wlogical-op # very verbose due to eigen
			-Wdouble-promotion
			-Werror=double-promotion
		)
	endif()

	if ($ENV{MEMORY_DEBUG} MATCHES "1")
		set(max_optimization -O0)

		set(optimization_flags
			-fno-strict-aliasing
			-fno-omit-frame-pointer
			-funsafe-math-optimizations
			-ffunction-sections
			-fdata-sections
			-g -fsanitize=address
			)
	else()
		set(max_optimization -Os)

		if ("${OS}" STREQUAL "qurt")
			set(PIC_FLAG -fPIC)
		endif()
		set(optimization_flags
			-fno-strict-aliasing
			-fomit-frame-pointer
			-funsafe-math-optimizations
			-ffunction-sections
			-fdata-sections
			${PIC_FLAG}
			)
	endif()

	if (NOT ${CMAKE_C_COMPILER_ID} MATCHES ".*Clang.*")
		list(APPEND optimization_flags
			-fno-strength-reduce
			-fno-builtin-printf
		)
	endif()

	set(c_warnings
		-Wbad-function-cast
		-Wstrict-prototypes
		-Wmissing-prototypes
		-Wnested-externs
		)

	if (NOT ${CMAKE_C_COMPILER_ID} MATCHES ".*Clang.*")
		list(APPEND c_warnings
			-Wold-style-declaration
			-Wmissing-parameter-type
		)
	endif()

	set(c_compile_flags
		-g
		-std=gnu99
		-fno-common
		)

	set(cxx_warnings
		-Wno-missing-field-initializers
		)

	set(cxx_compile_flags
		-g
		-fno-exceptions
		-fno-rtti
		-std=gnu++0x
		-fno-threadsafe-statics
		-DCONFIG_WCHAR_BUILTIN
		-D__CUSTOM_FILE_IO__
		)

	set(visibility_flags
		-fvisibility=hidden
		-include visibility.h
		)

	set(added_c_flags
		${c_compile_flags}
		${warnings}
		${c_warnings}
		${max_optimization}
		${optimization_flags}
		${visibility_flags}
		)

	set(added_cxx_flags
		${cxx_compile_flags}
		${warnings}
		${cxx_warnings}
		${max_optimization}
		${optimization_flags}
		${visibility_flags}
		)

	set(added_include_dirs
		${CMAKE_SOURCE_DIR}/src
		${CMAKE_BINARY_DIR}
		${CMAKE_BINARY_DIR}/src
		${CMAKE_SOURCE_DIR}/src/modules
		${CMAKE_SOURCE_DIR}/src/include
		${CMAKE_SOURCE_DIR}/src/lib
		${CMAKE_SOURCE_DIR}/src/platforms
		# TODO Build/versioning was in Makefile,
		# do we need this, how does it work with cmake
		${CMAKE_SOURCE_DIR}/src/drivers/boards/${BOARD}
		${CMAKE_BINARY_DIR}
		#${CMAKE_BINARY_DIR}/src/modules/px4_messages
		${CMAKE_BINARY_DIR}/src/modules
		#${CMAKE_SOURCE_DIR}/mavlink/include/mavlink
		${CMAKE_SOURCE_DIR}/src/lib/DriverFramework/framework/include
		)

	#list(APPEND added_include_dirs
	#	src/lib/matrix
	#	)

	set(added_link_dirs) # none used currently

	string(TOUPPER ${BOARD} board_upper)
	string(REPLACE "-" "_" board_config ${board_upper})
	set(added_definitions
		-DCONFIG_ARCH_BOARD_${board_config}
		)

	if (NOT ${CMAKE_C_COMPILER_ID} MATCHES ".*Clang.*")
		set(added_exe_link_flags
			-Wl,--warn-common
			-Wl,--gc-sections
			)
	else()
		set(added_exe_link_flags
			-Wl,--warn-common
			)
	endif()

	# output
	foreach(var ${inout_vars})
		string(TOLOWER ${var} lower_var)
		set(${${var}} ${${${var}}} ${added_${lower_var}} PARENT_SCOPE)
		#message(STATUS "set(${${var}} ${${${var}}} ${added_${lower_var}} PARENT_SCOPE)")
	endforeach()

endfunction()

#=============================================================================
#
#	px4_join
#
#	This function joins a list with a given separator. If list is not
#	passed, or is sent "", this will return the empty string.
#
#	Usage:
#		px4_join(OUT ${OUT} [ LIST ${LIST} ] GLUE ${GLUE})
#
#	Input:
#		LIST		: list to join
#		GLUE		: separator to use
#
#	Output:
#		OUT			: joined list
#
#	Example:
#		px4_join(OUT test_join LIST a b c GLUE ";")
#		test_join would then be:
#			"a;b;c"
#
function(px4_join)
	px4_parse_function_args(
		NAME px4_join
		ONE_VALUE OUT GLUE
		MULTI_VALUE LIST
		REQUIRED GLUE OUT
		ARGN ${ARGN})
	string (REPLACE ";" "${GLUE}" _TMP_STR "${LIST}")
	set(${OUT} ${_TMP_STR} PARENT_SCOPE)
endfunction()


#=============================================================================
#
#	px4_mangle_name
#
#	Convert a path name to a module name
#
#	Usage:
#		px4_mangle_name(dirname newname)
#
#	Input:
#		dirname					: path to module dir
#
#	Output:
#		newname					: module name
#
#	Example:
#		px4_mangle_name(${dirpath} mangled_name)
#		message(STATUS "module name is ${mangled_name}")
#
function(px4_mangle_name dirname newname)
	set(tmp)
	string(REPLACE "/" "__" tmp ${dirname})
	set(${newname} ${tmp} PARENT_SCOPE)
endfunction()

#=============================================================================
#
#	px4_add_upload
#
#	This function generates source code from ROS msg definitions.
#
#	Usage:
#		px4_add_upload(OUT <target> BUNDLE <file.px4>)
#
#	Input:
#		BUNDLE		: the firmware.px4 file
#		OS			: the operating system
#		BOARD		: the board
#
#	Output:
#		OUT			: the firmware target
#
#	Example:
#		px4_add_upload(OUT upload
#			BUNDLE main.px4
#			)
#
function(px4_add_upload)
	px4_parse_function_args(
		NAME px4_add_upload
		ONE_VALUE OS BOARD OUT BUNDLE
		REQUIRED OS BOARD OUT BUNDLE
		ARGN ${ARGN})
	set(serial_ports)
	if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")#主机选择
		list(APPEND serial_ports
			/dev/serial/by-id/usb-3D_Robotics*
			/dev/serial/by-id/pci-3D_Robotics*
			)
	elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Darwin")
		list(APPEND serial_ports
			/dev/tty.usbmodemPX*,/dev/tty.usbmodem*
			)
	elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows")
		foreach(port RANGE 32 0)
			list(APPEND serial_ports
				"COM${port}")
		endforeach()
	endif()
	px4_join(OUT serial_ports LIST "${serial_ports}" GLUE ",")#用，隔开
	add_custom_target(${OUT}
		COMMAND ${PYTHON_EXECUTABLE}
			${CMAKE_SOURCE_DIR}/Tools/px_uploader.py --port ${serial_ports} ${BUNDLE}
		DEPENDS ${BUNDLE}
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
		COMMENT "uploading ${BUNDLE}"
		VERBATIM
		USES_TERMINAL
		)
endfunction()


#=============================================================================
#
#	px4_generate_parameters_xml
#
#	Generates a parameters.xml file.
#
#	Usage:
#		px4_generate_parameters_xml(OUT <param-xml_file>)
#
#	Input:
#		BOARD : the board
#
#	Output:
#		OUT	: the generated xml file
#
#	Example:
#		px4_generate_parameters_xml(OUT parameters.xml)
#
function(px4_generate_parameters_xml)
	px4_parse_function_args(
		NAME px4_generate_parameters_xml
		ONE_VALUE OUT BOARD
		REQUIRED OUT BOARD
		ARGN ${ARGN})
	set(path ${CMAKE_SOURCE_DIR}/src)
	file(GLOB_RECURSE param_src_files
		${CMAKE_SOURCE_DIR}/src/*params.c
		)
	add_custom_command(OUTPUT ${OUT}
		COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Tools/px_process_params.py
			-s ${path} --board CONFIG_ARCH_${BOARD} --xml --inject-xml
		DEPENDS ${param_src_files}
		)
	set(${OUT} ${${OUT}} PARENT_SCOPE)
endfunction()

#=============================================================================
#
#	px4_generate_parameters_source

#
#	Generates a source file with all parameters.
#
#	Usage:
#		px4_generate_parameters_source(OUT <list-source-files> XML <param-xml-file>)
#
#	Input:
#		XML : the parameters.xml file
#		DEPS : target dependencies
#
#	Output:
#		OUT	: the generated source files
#
#	Example:
#		px4_generate_parameters_source(OUT param_files XML parameters.xml)
#
function(px4_generate_parameters_source)
	px4_parse_function_args(
		NAME px4_generate_parameters_source
		ONE_VALUE OUT XML DEPS
		REQUIRED OUT XML
		ARGN ${ARGN})
	set(generated_files
		${CMAKE_CURRENT_BINARY_DIR}/px4_parameters.h
		${CMAKE_CURRENT_BINARY_DIR}/px4_parameters.c)
	set_source_files_properties(${generated_files}
		PROPERTIES GENERATED TRUE)
	add_custom_command(OUTPUT ${generated_files}
		COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Tools/px_generate_params.py ${XML}
		DEPENDS ${XML} ${DEPS}
		)
	set(${OUT} ${generated_files} PARENT_SCOPE)
endfunction()

#=============================================================================
#
#	px4_generate_airframes_xml
#
#	Generates airframes.xml
#
#	Usage:
#		px4_generate_airframes_xml(OUT <airframe-xml-file>)
#
#	Input:
#		XML : the airframes.xml file
#		BOARD : the board
#
#	Output:
#		OUT	: the generated source files
#
#	Example:
#		px4_generate_airframes_xml(OUT airframes.xml)
#
function(px4_generate_airframes_xml)
	px4_parse_function_args(
		NAME px4_generate_airframes_xml
		ONE_VALUE OUT BOARD
		REQUIRED OUT BOARD
		ARGN ${ARGN})
	set(process_airframes ${CMAKE_SOURCE_DIR}/Tools/px_process_airframes.py)
	add_custom_command(OUTPUT ${OUT}
		COMMAND ${PYTHON_EXECUTABLE} ${process_airframes}
			-a ${CMAKE_SOURCE_DIR}/ROMFS/px4fmu_common/init.d
			--board CONFIG_ARCH_BOARD_${BOARD} --xml
		)
	set(${OUT} ${${OUT}} PARENT_SCOPE)
endfunction()


