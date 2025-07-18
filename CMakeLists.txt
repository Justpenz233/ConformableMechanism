cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 20)

project(ConformableMechanism)

set(BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR}/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BIN_DIR}/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BIN_DIR}/${CMAKE_BUILD_TYPE})

add_subdirectory(Engine/ThirdParty)
add_subdirectory(ConformableMechanism)
add_subdirectory(Engine)


add_executable(ConformableMechanism main.cpp)
target_link_libraries(ConformableMechanism PUBLIC MechEngineEditor)
target_compile_definitions(ConformableMechanism PRIVATE "PROJECT_DIR=\"${USER_PROJECT_ROOT_DIR}\"")