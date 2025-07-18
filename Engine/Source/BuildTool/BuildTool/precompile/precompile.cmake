set(PRECOMPILE_TOOLS_PATH "${BUILD_TOOL_DIR}/bin")
set(PICCOLO_PRECOMPILE_PARAMS_IN_PATH "${BUILD_TOOL_DIR}/precompile/precompile.json.in")
set(PICCOLO_PRECOMPILE_PARAMS_PATH "${PRECOMPILE_TOOLS_PATH}/precompile.json")
# Replace json.in to .json  with our headers
configure_file(${PICCOLO_PRECOMPILE_PARAMS_IN_PATH} ${PICCOLO_PRECOMPILE_PARAMS_PATH})

#
# use wine for linux
if (CMAKE_HOST_WIN32)
    set(PRECOMPILE_PRE_EXE)
	set(PRECOMPILE_PARSER ${PRECOMPILE_TOOLS_PATH}/PiccoloParser.exe)
    set(sys_include "*") 
elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux" )
    set(PRECOMPILE_PRE_EXE)
	set(PRECOMPILE_PARSER ${PRECOMPILE_TOOLS_PATH}/PiccoloParser)
    set(sys_include "${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES}")
    string(JOIN "," sys_include ${sys_include})
    message(STATUS "sys_include: ${sys_include}")
    #execute_process(COMMAND chmod a+x ${PRECOMPILE_PARSER} WORKING_DIRECTORY ${PRECOMPILE_TOOLS_PATH})
elseif(CMAKE_HOST_APPLE)
    set(PRECOMPILE_PRE_EXE)
    set(PRECOMPILE_PARSER ${PRECOMPILE_TOOLS_PATH}/PiccoloParser)
    set(sys_include "${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES}")
    string(JOIN "," sys_include ${sys_include})
    message(STATUS "sys_include: ${sys_include}")
endif()

set (PARSER_INPUT ${CMAKE_BINARY_DIR}/parser_header.h)
set(PRECOMPILE_TARGET "PiccoloPreCompile")

add_custom_target(${PRECOMPILE_TARGET} ALL
        COMMAND
        ${PRECOMPILE_PARSER} "${PICCOLO_PRECOMPILE_PARAMS_PATH}" "${PARSER_INPUT}" ${INCLUDE_DIRS} ${sys_include} "MECHENGINE" 0 ${BUILD_TOOL_DIR} ${GENERATED_DIR}
)