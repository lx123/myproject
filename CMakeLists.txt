#cmake版本控制
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#====================================================
#参数设置
#
set(CMAKE_BUILD_TYPE "" CACHE STRING "build type")
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
	STRINGS ";Debug;Release;RelWithDebInfo;MinSizeRel")
set(CONFIG "nuttx_fmu_default" CACHE STRING "desired configuration")
file(GLOB_RECURSE configs RELATIVE cmake/configs "cmake/configs/*.cmake")
set_property(CACHE CONFIG PROPERTY STRINGS ${configs})
set(THREADS "4" CACHE STRING
	"number of threads to use for external build processes")
set(DEBUG_PORT "/dev/ttyACM0" CACHE STRING "debugging port")

#====================================================
#配置整个系统，系统类型，板子类型等
#
string(REPLACE "_" ";" config_args ${CONFIG})  #把CONFIG中字符串用；代替_，并保存到config_args
list(GET config_args 0 OS)  #对字符串列表操作nuttx
list(GET config_args 1 BOARD)#fmu
list(GET config_args 2 LABEL)#default
set(target_name "${OS}-${BOARD}-${LABEL}") #设置目标名称，nuttx_fmu_default
message(STATUS "${target_name}")  #打印目标名称

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)#cmake的位置
set(config_module "configs/${CONFIG}")#nuttx_fmu_default.cmake包含
include(${config_module})

#cmake 模块
include(ExternalProject)

# require px4 module interface
set(px4_required_interface
	px4_os_prebuild_targets
	px4_os_add_flags
	)
foreach(cmd ${px4_required_interface})
	if(NOT COMMAND ${cmd})
		message(FATAL_ERROR "${config_module} must implement ${cmd}")
	endif()
endforeach()
set(px4_required_config
	config_module_list
	)
foreach(conf ${px4_required_config})
	if(NOT DEFINED ${conf})
		message(FATAL_ERROR "cmake/${config_module} must define ${conf}")
	endif()
endforeach()

#====================================================
#项目定义
#
project(fmu CXX C ASM)
if (NOT ${CMAKE_VERSION} VERSION_LESS 3.0.0)  #如果cmake的版本高于3.0
	cmake_policy(SET CMP0045 NEW) # error on non-existent target in get prop
	cmake_policy(SET CMP0046 NEW) # no non-existent targets as dependencies
	cmake_policy(SET CMP0025 OLD) # still report AppleClang as Clang
endif()
if (NOT ${CMAKE_VERSION} VERSION_LESS 3.1.0)  #cmake高于3.1
	cmake_policy(SET CMP0054 NEW) # don't dereference quoted variables
endif()

set(version_major 1)
set(version_minor 0)
set(version_patch 1)
set(version "${version_major}.${version_minor}.${version_patch}")
message(STATUS "SoftVersion=${version}") 

#=============================================================================
# programs
#
find_package(PythonInterp REQUIRED)

#=============================================================================
# cmake modules
#
enable_testing()

#=============================================================================
# check required toolchain variables
#
set(required_variables
	CMAKE_C_COMPILER_ID
	)
foreach(var ${required_variables})
	if (NOT ${var})
		message(FATAL_ERROR "Toolchain/config must define ${var}")
	endif()
endforeach()

#=============================================================================
# external libraries
#
px4_os_prebuild_targets(OUT prebuild_targets
	BOARD ${BOARD}#nuttx
	THREADS ${THREADS})#4

#=============================================================================
# build flags
#
px4_os_add_flags(
	BOARD ${BOARD}#nuttx
	C_FLAGS c_flags
	CXX_FLAGS cxx_flags
	EXE_LINKER_FLAGS exe_linker_flags
	INCLUDE_DIRS include_dirs
	LINK_DIRS link_dirs
	DEFINITIONS definitions)

px4_join(OUT CMAKE_EXE_LINKER_FLAGS LIST "${exe_linker_flags}" GLUE " ")
px4_join(OUT CMAKE_C_FLAGS LIST "${c_flags}" GLUE " ")
px4_join(OUT CMAKE_CXX_FLAGS LIST "${cxx_flags}" GLUE " ")

include_directories(${include_dirs})
message("INCLUDE_DIRS=${include_dirs}")
link_directories(${link_dirs})
add_definitions(${definitions})

#=============================================================================
# source code generation
#
#file(GLOB_RECURSE msg_files msg/*.msg)
#px4_generate_messages(TARGET msg_gen
#	MSG_FILES ${msg_files}
#	OS ${OS}
#	DEPENDS git_genmsg git_gencpp
#	)
#px4_generate_parameters_xml(OUT parameters.xml BOARD ${BOARD})
#px4_generate_airframes_xml(OUT airframes.xml BOARD ${BOARD})
#add_custom_target(xml_gen
#	DEPENDS parameters.xml airframes.xml)

#=============================================================================
# external projects
#

set(ep_base ${CMAKE_BINARY_DIR}/external)
set_property(DIRECTORY PROPERTY EP_BASE ${ep_base})

# add external project install folders to build
link_directories(${ep_base}/Install/lib)
include_directories(${ep_base}/Install/include)
# add the directories so cmake won't warn
execute_process(COMMAND cmake -E make_directory ${ep_base}/Install/lib)
execute_process(COMMAND cmake -E make_directory ${ep_base}/Install/include)

#=============================================================================
# subdirectories
#
set(module_libraries)
foreach(module ${config_module_list})
	string(REGEX MATCH "^[./]" external_module ${module})
	if(external_module)
		STRING(REGEX REPLACE "//" "/" EXT_MODULE ${module})
		STRING(REGEX REPLACE "/" "__" EXT_MODULE_PREFIX ${EXT_MODULE})
		add_subdirectory(${module} ${CMAKE_BINARY_DIR}/${EXT_MODULE_PREFIX})
	else()
		add_subdirectory(src/${module})
	endif()
	px4_mangle_name(${module} mangled_name)
	list(APPEND module_libraries ${mangled_name})
	#message(STATUS "adding module: ${module}")
endforeach()

add_subdirectory(src/firmware/${OS})

#add_subdirectory(src/lib/DriverFramework/framework/src)

#add_dependencies(df_driver_framework nuttx_export_${CONFIG}.stamp)
#if (NOT "${OS}" STREQUAL "nuttx")
#endif()

#if (config_io_board)
#	add_subdirectory(src/modules/px4iofirmware)
#endif()

#=============================================================================
# packaging
#
# Important to having packaging at end of cmake file.
#
set(CPACK_PACKAGE_VERSION ${version})
set(CPACK_PACKAGE_CONTACT ${package_contact})
set(CPACK_GENERATOR "ZIP")
set(CPACK_SOURCE_GENERATOR "ZIP")
include(CPack)









