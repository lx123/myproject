px4_nuttx_generate_builtin_commands(  #构建内部命令
	OUT builtin_commands.c
	MODULE_LIST
		${module_libraries}
		${config_extra_builtin_cmds}
	)

# add executable
add_executable(firmware_nuttx#定义输出名称
	builtin_commands.c)



set(nuttx_export_dir ${CMAKE_BINARY_DIR}/${BOARD}/NuttX/nuttx-export)

set(link_libs#链接的库文件
	 romfs apps nuttx m gcc# df_driver_framework
	)

if(NOT ${BOARD} STREQUAL "sim")
	list(APPEND link_libs nosys)
	set(main_link_flags
		"-T${nuttx_export_dir}/build/ld.script"
		"-Wl,-Map=${CMAKE_BINARY_DIR}/${BOARD}/main.map"
		)
	px4_join(OUT main_link_flags LIST ${main_link_flags} GLUE " ")
	set_target_properties(firmware_nuttx PROPERTIES LINK_FLAGS ${main_link_flags})
endif()

target_link_libraries(firmware_nuttx
	-Wl,--start-group
	${module_libraries}
	${config_extra_libs}
	${link_libs}
	-Wl,--end-group)

if(NOT ${BOARD} STREQUAL "sim")

#	if (config_io_board)
#		set(extras "${CMAKE_BINARY_DIR}/src/modules/px4iofirmware/${config_io_board}.bin")
#	endif()

	px4_nuttx_add_romfs(OUT romfs
		ROOT ROMFS/fmu_common
		EXTRAS ${extras}
		)
#	if (config_io_board)
#	    add_dependencies(romfs fw_io)#依赖关系
#	endif()
	set(fw_file
		${CMAKE_CURRENT_BINARY_DIR}/${OS}-${BOARD}-${LABEL}.px4)

	px4_nuttx_add_firmware(OUT ${fw_file}
		BOARD ${BOARD}
		EXE ${CMAKE_CURRENT_BINARY_DIR}/firmware_nuttx
		PARAM_XML ${CMAKE_BINARY_DIR}/parameters.xml
		AIRFRAMES_XML ${CMAKE_BINARY_DIR}/airframes.xml
		)
#将一份文件修改，并转存到另一份文件。将文件<input>拷贝到<output>然后替换文件内容中引用到的变量值。
#	configure_file(gdbinit.in .gdbinit)

#	add_custom_target(debug
#		COMMAND ${GDB} ${CMAKE_CURRENT_BINARY_DIR}/firmware_nuttx
#		DEPENDS firmware_nuttx
#			${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
#		)

#	add_custom_target(debug_tui
#		COMMAND ${GDBTUI} ${CMAKE_CURRENT_BINARY_DIR}/firmware_nuttx
#		DEPENDS firmware_nuttx
#			${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
#		)

#	add_custom_target(debug_ddd
#		COMMAND ${DDD} --debugger ${GDB} ${CMAKE_CURRENT_BINARY_DIR}/firmware_nuttx
#		DEPENDS firmware_nuttx
#			${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
#		)

#	add_custom_target(debug_io
#		COMMAND ${GDB}
#			${CMAKE_BINARY_DIR}/src/modules/px4iofirmware/${config_io_board}
#		DEPENDS firmware_nuttx
#			${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
#		)

#	add_custom_target(debug_io_tui
#		COMMAND ${GDBTUI}
#			${CMAKE_BINARY_DIR}/src/modules/px4iofirmware/${config_io_board}
#		DEPENDS firmware_nuttx
#			${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
#		)

#	add_custom_target(debug_io_ddd
#		COMMAND ${DDD} --debugger ${GDB}
#			${CMAKE_BINARY_DIR}/src/modules/px4iofirmware/${config_io_board}
#		DEPENDS firmware_nuttx
#			${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
#		)

#	px4_add_upload(OUT upload OS ${OS} BOARD ${BOARD}
#		BUNDLE ${fw_file})
endif()

install(FILES ${fw_file} DESTINATION .)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
