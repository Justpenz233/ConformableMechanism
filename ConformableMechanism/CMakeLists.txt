cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 17)

project(ConformableMechanismLib)
set(USER_PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
set(USER_PROJECT_SOURCE_DIR ${PROJECT_ROOT_DIR}/Source PARENT_SCOPE)
set(USER_PROJECT_NAME ${PROJECT_NAME} PARENT_SCOPE)

file(GLOB_RECURSE SOURCE_FILES Source/*.cpp)
file(GLOB_RECURSE HEADER_FILES Source/*.h)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${HEADER_FILES} ${SOURCE_FILES})

add_library(ConformableMechanismLib SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(ConformableMechanismLib PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
set_target_properties(ConformableMechanismLib PROPERTIES PARSE_HEADERS "${HEADER_FILES}")
target_include_directories(ConformableMechanismLib PUBLIC Source/)
target_link_libraries(ConformableMechanismLib PUBLIC MechEngineRuntime)
add_subdirectory(3rd)
target_link_libraries(ConformableMechanismLib PUBLIC pagmo)
target_link_libraries(ConformableMechanismLib PUBLIC hpp-fcl)
