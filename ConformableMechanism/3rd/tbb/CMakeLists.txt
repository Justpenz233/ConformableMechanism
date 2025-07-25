cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

if (POLICY CMP0048)
  # cmake warns if loaded from a min-3.0-required parent dir, so silence the warning:
  cmake_policy(SET CMP0048 NEW)
endif()

project (tbb CXX)

include(CheckCXXCompilerFlag)
include(CheckCXXSourceRuns)

if(POLICY CMP0058)
  cmake_policy(SET CMP0058 NEW)
endif()

if(POLICY CMP0068)
  cmake_policy(SET CMP0068 NEW)
endif()

if (POLICY CMP0078)
  # swig standard target names
  cmake_policy(SET CMP0078 NEW)
endif ()

if (POLICY CMP0086)
  # UseSWIG honors SWIG_MODULE_NAME via -module flag
  cmake_policy(SET CMP0086 NEW)
endif ()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

if(NOT TBB_INSTALL_RUNTIME_DIR)
  set(TBB_INSTALL_RUNTIME_DIR bin)
endif()
if(NOT TBB_INSTALL_LIBRARY_DIR)
  set(TBB_INSTALL_LIBRARY_DIR lib)
endif()
if(NOT TBB_INSTALL_ARCHIVE_DIR)
  set(TBB_INSTALL_ARCHIVE_DIR lib)
endif()
if(NOT TBB_INSTALL_INCLUDE_DIR)
  set(TBB_INSTALL_INCLUDE_DIR include)
endif()
if(NOT TBB_CMAKE_PACKAGE_INSTALL_DIR)
  set(TBB_CMAKE_PACKAGE_INSTALL_DIR lib/cmake/tbb)
endif()

include_directories(include src src/rml/include ${CMAKE_CURRENT_BINARY_DIR})

option(TBB_BUILD_SHARED          "Build TBB shared library" ON)
option(TBB_BUILD_STATIC          "Build TBB static library" ON)
option(TBB_BUILD_TBBMALLOC       "Build TBB malloc library" ON)
option(TBB_BUILD_TBBMALLOC_PROXY "Build TBB malloc proxy library" ON)
option(TBB_BUILD_TESTS           "Build TBB tests and enable testing infrastructure" OFF)
option(TBB_NO_DATE               "Do not save the configure date in the version string" OFF)
option(TBB_BUILD_PYTHON          "Build TBB Python bindings" OFF)
option(TBB_SET_SOVERSION         "Set the SOVERSION (shared library build version suffix)?" OFF)

# When this repository is part of a larger build system of a parent project
# we may not want TBB to set up default installation targets
option(TBB_INSTALL_TARGETS       "Include build targets for 'make install'" OFF)

if(APPLE)
  set(CMAKE_MACOSX_RPATH ON)
endif()

file(GLOB tbb_src "${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/old/*.cpp")
list(REMOVE_ITEM tbb_src ${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/tbb_bind.cpp)
list(APPEND tbb_src ${CMAKE_CURRENT_SOURCE_DIR}/src/rml/client/rml_tbb.cpp)
file(GLOB to_remove "${CMAKE_CURRENT_SOURCE_DIR}/src/old/test*.cpp")
list(REMOVE_ITEM tbb_src ${to_remove})

set(tbbmalloc_static_src
  src/tbbmalloc/backend.cpp
  src/tbbmalloc/large_objects.cpp
  src/tbbmalloc/backref.cpp
  src/tbbmalloc/tbbmalloc.cpp
  src/tbbmalloc/frontend.cpp
  src/tbb/itt_notify.cpp)

set(tbbmalloc_src ${tbbmalloc_static_src})

set(tbbmalloc_proxy_src
  src/tbbmalloc/proxy.cpp
  src/tbbmalloc/tbb_function_replacement.cpp)

add_library (tbb_interface INTERFACE)
add_definitions(-DTBB_SUPPRESS_DEPRECATED_MESSAGES=1)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "(i386|x86_64)")
  if (NOT APPLE AND NOT MINGW)
    target_compile_definitions(tbb_interface INTERFACE DO_ITT_NOTIFY)
  endif()
endif()

if (APPLE)
  # Disable annoying "has no symbols" warnings
  set(CMAKE_C_ARCHIVE_CREATE   "<CMAKE_AR> Scr <TARGET> <LINK_FLAGS> <OBJECTS>")
  set(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> Scr <TARGET> <LINK_FLAGS> <OBJECTS>")
  set(CMAKE_C_ARCHIVE_FINISH   "<CMAKE_RANLIB> -no_warning_for_no_symbols -c <TARGET>")
  set(CMAKE_CXX_ARCHIVE_FINISH "<CMAKE_RANLIB> -no_warning_for_no_symbols -c <TARGET>")
endif()

macro(CHECK_CXX_COMPILER_AND_LINKER_FLAGS _RESULT _CXX_FLAGS _LINKER_FLAGS)
  set(CMAKE_REQUIRED_FLAGS ${_CXX_FLAGS})
  set(CMAKE_REQUIRED_LIBRARIES ${_LINKER_FLAGS})
  set(CMAKE_REQUIRED_QUIET TRUE)
  check_cxx_source_runs("#include <iostream>\nint main(int argc, char **argv) { std::cout << \"test\"; return 0; }" ${_RESULT})
  set(CMAKE_REQUIRED_FLAGS "")
  set(CMAKE_REQUIRED_LIBRARIES "")
endmacro()

# Prefer libc++ in conjunction with Clang
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if (CMAKE_CXX_FLAGS MATCHES "-stdlib=libc\\+\\+")
    message(STATUS "TBB: using libc++.")
  else()
    CHECK_CXX_COMPILER_AND_LINKER_FLAGS(HAS_LIBCPP "-stdlib=libc++" "-stdlib=libc++")
    if (HAS_LIBCPP)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++ -D_LIBCPP_VERSION")
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libc++")
      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -stdlib=libc++")
      message(STATUS "TBB: using libc++.")
    else()
      message(STATUS "TBB: NOT using libc++.")
    endif()
  endif()
endif()

set (CMAKE_CXX_STANDARD 11)

if (UNIX)
  target_compile_definitions(tbb_interface INTERFACE USE_PTHREAD)

  check_cxx_compiler_flag ("-mrtm -Werror" SUPPORTS_MRTM)
  if (SUPPORTS_MRTM)
    target_compile_options(tbb_interface INTERFACE "-mrtm")
  endif ()

elseif(WIN32)
  target_compile_definitions(tbb_interface INTERFACE USE_WINTHREAD _WIN32_WINNT=0x0600)
  if (MSVC)
    enable_language(ASM_MASM)
    target_compile_options(tbb_interface INTERFACE /GS- /Zc:wchar_t /Zc:forScope)
    check_cxx_compiler_flag ("/volatile:iso" SUPPORTS_VOLATILE_FLAG)
    if (SUPPORTS_VOLATILE_FLAG)
      target_compile_options(tbb_interface INTERFACE /volatile:iso)
    endif ()
    target_compile_options(tbb_interface INTERFACE $<$<COMPILE_LANGUAGE:CXX>:/wd4267 /wd4800 /wd4146 /wd4244 /wd4577 /wd4018>)
    if (NOT CMAKE_SIZEOF_VOID_P)
       message(FATAL_ERROR "'CMAKE_SIZEOF_VOID_P' is undefined. Please delete your build directory and rerun CMake again!")
    endif()

    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
      list(APPEND tbb_src src/tbb/intel64-masm/atomic_support.asm
        src/tbb/intel64-masm/itsx.asm src/tbb/intel64-masm/intel64_misc.asm)
      set_source_files_properties(
        src/tbb/intel64-masm/atomic_support.asm
        src/tbb/intel64-masm/itsx.asm src/tbb/intel64-masm/intel64_misc.asm
        src/tbb/intel64-masm/atomic_support.asm
        PROPERTIES LANGUAGE ASM_MASM)
      list(APPEND tbbmalloc_src src/tbb/intel64-masm/atomic_support.asm)
      set(CMAKE_ASM_MASM_FLAGS "/DEM64T=1 ${CMAKE_ASM_MASM_FLAGS}")
    else()
      list(APPEND tbb_src src/tbb/ia32-masm/atomic_support.asm
        src/tbb/ia32-masm/itsx.asm src/tbb/ia32-masm/lock_byte.asm)
      set_source_files_properties(
        src/tbb/ia32-masm/atomic_support.asm
        src/tbb/ia32-masm/itsx.asm src/tbb/ia32-masm/lock_byte.asm
        PROPERTIES LANGUAGE ASM_MASM)
      # Enable SAFESEH feature for assembly (x86 builds only).
      set(CMAKE_ASM_MASM_FLAGS "/safeseh ${CMAKE_ASM_MASM_FLAGS}")
    endif()
  elseif (MINGW)
    target_compile_options(tbb_interface INTERFACE "-mthreads")
  endif ()
endif()

if (MSVC)
  set(ENABLE_RTTI "/EHsc /GR ")
  set(DISABLE_RTTI "/EHs- /GR- ")
elseif (UNIX)
  set(ENABLE_RTTI "-frtti -fexceptions ")
  set(DISABLE_RTTI "-fno-rtti -fno-exceptions ")
endif ()

##--------
#   - Added TBB_USE_GLIBCXX_VERSION macro to specify the version of GNU
#     libstdc++ when it cannot be properly recognized, e.g. when used
#     with Clang on Linux* OS. Inspired by a contribution from David A.
if (NOT TBB_USE_GLIBCXX_VERSION AND UNIX AND NOT APPLE)
  if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    # using Clang
    string(REPLACE "." "0" TBB_USE_GLIBCXX_VERSION ${CMAKE_CXX_COMPILER_VERSION})
  endif()
endif()

if (TBB_USE_GLIBCXX_VERSION)
  target_compile_definitions(tbb_interface INTERFACE TBB_USE_GLIBCXX_VERSION=${TBB_USE_GLIBCXX_VERSION})
endif()

##-------

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
   check_cxx_compiler_flag ("-flifetime-dse=1" SUPPORTS_FLIFETIME)
   if (SUPPORTS_FLIFETIME)
     target_compile_options(tbb_interface INTERFACE -flifetime-dse=1)
   endif()
endif()

# Linker export definitions
if (APPLE)
  set (ARCH_PREFIX "mac")
elseif(WIN32)
  set (ARCH_PREFIX "win")
else()
  set (ARCH_PREFIX "lin")
endif()

if (CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ARCH_PREFIX "${ARCH_PREFIX}64")
else()
  set(ARCH_PREFIX "${ARCH_PREFIX}32")
endif()

if (MINGW)
  set (ARCH_PREFIX "${ARCH_PREFIX}-gcc")
  # there's no win32-gcc-tbb-export.def, use lin32-tbb-export.def
  execute_process (COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/lin32-tbb-export.def ${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/win32-gcc-tbb-export.def)
endif()

if (MSVC)
  add_custom_command(OUTPUT tbb.def
    COMMAND ${CMAKE_CXX_COMPILER} /TC /EP ${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/${ARCH_PREFIX}-tbb-export.def  -I ${CMAKE_CURRENT_SOURCE_DIR}/include > tbb.def
    MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/${ARCH_PREFIX}-tbb-export.def
    COMMENT "Preprocessing tbb.def"
  )

  add_custom_command(OUTPUT tbbmalloc.def
    COMMAND ${CMAKE_CXX_COMPILER} /TC /EP ${CMAKE_CURRENT_SOURCE_DIR}/src/tbbmalloc/${ARCH_PREFIX}-tbbmalloc-export.def  -I ${CMAKE_CURRENT_SOURCE_DIR}/include >   tbbmalloc.def
    MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/src/tbbmalloc/${ARCH_PREFIX}-tbbmalloc-export.def
    COMMENT "Preprocessing tbbmalloc.def"
  )
else()
  add_custom_command(OUTPUT tbb.def
    COMMAND ${CMAKE_CXX_COMPILER} -xc++ -std=c++11 -E ${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/${ARCH_PREFIX}-tbb-export.def  -I ${CMAKE_CURRENT_SOURCE_DIR}/include -o tbb.def
    MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/${ARCH_PREFIX}-tbb-export.def
    COMMENT "Preprocessing tbb.def"
  )

  add_custom_command(OUTPUT tbbmalloc.def
    COMMAND ${CMAKE_CXX_COMPILER} -xc++ -std=c++11 -E ${CMAKE_CURRENT_SOURCE_DIR}/src/tbbmalloc/${ARCH_PREFIX}-tbbmalloc-export.def  -I ${CMAKE_CURRENT_SOURCE_DIR}/include -o   tbbmalloc.def
    MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/src/tbbmalloc/${ARCH_PREFIX}-tbbmalloc-export.def
    COMMENT "Preprocessing tbbmalloc.def"
  )
endif()

add_custom_target(tbb_def_files DEPENDS tbb.def tbbmalloc.def)

# TBB library
if (TBB_BUILD_STATIC)
  add_library(tbb_static STATIC ${tbb_src})
  target_link_libraries(tbb_static PRIVATE tbb_interface)
  target_include_directories(tbb_static INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<INSTALL_INTERFACE:${TBB_INSTALL_INCLUDE_DIR}>")
  set_property(TARGET tbb_static APPEND_STRING PROPERTY COMPILE_FLAGS ${ENABLE_RTTI})
  if (TBB_INSTALL_TARGETS)
    install(TARGETS tbb_static ARCHIVE DESTINATION ${TBB_INSTALL_ARCHIVE_DIR})
  endif()

  target_compile_definitions(tbb_static
    PRIVATE
      -D__TBB_BUILD=1
      -D__TBB_DYNAMIC_LOAD_ENABLED=0
      -D__TBB_SOURCE_DIRECTLY_INCLUDED=1)

  if (MSVC)
    target_compile_definitions(tbb_static
      PUBLIC -D__TBB_NO_IMPLICIT_LINKAGE=1
      PRIVATE -D_CRT_SECURE_NO_WARNINGS)
  endif()

  if (UNIX AND NOT APPLE)
    target_link_libraries(tbb_static PUBLIC pthread dl)
  endif()
endif()

if (TBB_BUILD_SHARED)
  add_library(tbb SHARED ${tbb_src})
  target_link_libraries(tbb PRIVATE tbb_interface)
  target_include_directories(tbb INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<INSTALL_INTERFACE:${TBB_INSTALL_INCLUDE_DIR}>")
  set_property(TARGET tbb APPEND_STRING PROPERTY COMPILE_FLAGS ${ENABLE_RTTI})
  if (TBB_SET_SOVERSION)
    set_property(TARGET tbb PROPERTY SOVERSION 2)
  endif ()

  target_compile_definitions(tbb
    PRIVATE -D__TBB_BUILD=1)

  if (MSVC)
    target_compile_definitions(tbb
      PUBLIC -D__TBB_NO_IMPLICIT_LINKAGE=1
      PRIVATE -D_CRT_SECURE_NO_WARNINGS)
  endif()

  add_dependencies(tbb tbb_def_files)

  if (APPLE)
    set_property(TARGET tbb APPEND PROPERTY LINK_FLAGS "-Wl,-exported_symbols_list,\"${CMAKE_CURRENT_BINARY_DIR}/tbb.def\"")
  elseif (MSVC)
    set_property(TARGET tbb APPEND PROPERTY LINK_FLAGS "/DEF:\"${CMAKE_CURRENT_BINARY_DIR}/tbb.def\"")
  else ()
    set_property(TARGET tbb APPEND PROPERTY LINK_FLAGS "-Wl,-version-script,\"${CMAKE_CURRENT_BINARY_DIR}/tbb.def\"")
  endif()

  if (TBB_INSTALL_TARGETS)
    install(TARGETS tbb EXPORT TBB
            LIBRARY DESTINATION ${TBB_INSTALL_LIBRARY_DIR}
            ARCHIVE DESTINATION ${TBB_INSTALL_ARCHIVE_DIR}
            RUNTIME DESTINATION ${TBB_INSTALL_RUNTIME_DIR})
    if (MSVC)
      install(FILES $<TARGET_PDB_FILE:tbb> DESTINATION ${TBB_INSTALL_RUNTIME_DIR} OPTIONAL)
    endif()
  endif()

  if (UNIX AND NOT APPLE)
    target_link_libraries(tbb PUBLIC pthread dl)
  endif()
endif()


if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  # Quench a warning on GCC
  set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/governor.cpp COMPILE_FLAGS "-Wno-missing-field-initializers ")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  # Quench a warning on Clang
  set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/itt_notify.cpp COMPILE_FLAGS "-Wno-varargs ")
elseif(MSVC)
  # Quench a warning on MSVC
  set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/scheduler.cpp COMPILE_FLAGS "/wd4458 ")
endif()

if(TBB_BUILD_TBBMALLOC)
  # TBB malloc library
  if (TBB_BUILD_STATIC)
    add_library(tbbmalloc_static STATIC ${tbbmalloc_static_src})
    target_link_libraries(tbbmalloc_static PRIVATE tbb_interface)
    set_property(TARGET tbbmalloc_static APPEND PROPERTY COMPILE_DEFINITIONS "__TBBMALLOC_BUILD=1")
    set_property(TARGET tbbmalloc_static APPEND PROPERTY COMPILE_DEFINITIONS "__TBB_DYNAMIC_LOAD_ENABLED=0")
    set_property(TARGET tbbmalloc_static APPEND PROPERTY COMPILE_DEFINITIONS "__TBB_SOURCE_DIRECTLY_INCLUDED=1")
    set_property(TARGET tbbmalloc_static APPEND_STRING PROPERTY COMPILE_FLAGS ${DISABLE_RTTI})
    if (MSVC)
      target_compile_definitions(tbbmalloc_static PUBLIC __TBB_NO_IMPLICIT_LINKAGE=1 __TBBMALLOC_NO_IMPLICIT_LINKAGE=1)
    endif()
    if (TBB_INSTALL_TARGETS)
      install(TARGETS tbbmalloc_static ARCHIVE DESTINATION ${TBB_INSTALL_ARCHIVE_DIR})
    endif()
  endif()

  if (TBB_BUILD_SHARED)
    add_library(tbbmalloc SHARED ${tbbmalloc_src})
    target_link_libraries(tbbmalloc PRIVATE tbb_interface)
    set_property(TARGET tbbmalloc APPEND PROPERTY COMPILE_DEFINITIONS "__TBBMALLOC_BUILD=1")
    set_property(TARGET tbbmalloc APPEND_STRING PROPERTY COMPILE_FLAGS ${DISABLE_RTTI})
    if (TBB_SET_SOVERSION)
      set_property(TARGET tbbmalloc PROPERTY SOVERSION 2)
    endif ()
    add_dependencies(tbbmalloc tbb_def_files)
    if (APPLE)
      set_property(TARGET tbbmalloc APPEND PROPERTY LINK_FLAGS "-Wl,-exported_symbols_list,\"${CMAKE_CURRENT_BINARY_DIR}/tbbmalloc.def\"")
    elseif (MSVC)
      set_property(TARGET tbbmalloc APPEND PROPERTY LINK_FLAGS "/DEF:\"${CMAKE_CURRENT_BINARY_DIR}/tbbmalloc.def\"")
    else ()
      set_property(TARGET tbbmalloc APPEND PROPERTY LINK_FLAGS "-Wl,-version-script,\"${CMAKE_CURRENT_BINARY_DIR}/tbbmalloc.def\"")
    endif()
    if (MSVC)
      target_compile_definitions(tbbmalloc PUBLIC __TBB_NO_IMPLICIT_LINKAGE=1 __TBBMALLOC_NO_IMPLICIT_LINKAGE=1)
    endif()
    if (TBB_INSTALL_TARGETS)
      install(TARGETS tbbmalloc EXPORT TBB
              LIBRARY DESTINATION ${TBB_INSTALL_LIBRARY_DIR}
              ARCHIVE DESTINATION ${TBB_INSTALL_ARCHIVE_DIR}
              RUNTIME DESTINATION ${TBB_INSTALL_RUNTIME_DIR})
      if (MSVC)
        install(FILES $<TARGET_PDB_FILE:tbbmalloc> DESTINATION ${TBB_INSTALL_RUNTIME_DIR} OPTIONAL)
      endif()
    endif()
    if (UNIX AND NOT APPLE)
      target_link_libraries(tbbmalloc PUBLIC pthread dl)
    endif()
  endif()
endif()

if(TBB_BUILD_TBBMALLOC_PROXY)
  # TBB malloc proxy library
  if (TBB_BUILD_STATIC)
    add_library(tbbmalloc_proxy_static STATIC ${tbbmalloc_proxy_src})
    target_link_libraries(tbbmalloc_proxy_static PRIVATE tbb_interface)
    set_property(TARGET tbbmalloc_proxy_static APPEND PROPERTY COMPILE_DEFINITIONS "__TBBMALLOC_BUILD=1")
    set_property(TARGET tbbmalloc_proxy_static APPEND PROPERTY COMPILE_DEFINITIONS "__TBB_DYNAMIC_LOAD_ENABLED=0")
    set_property(TARGET tbbmalloc_proxy_static APPEND PROPERTY COMPILE_DEFINITIONS "__TBB_SOURCE_DIRECTLY_INCLUDED=1")
    set_property(TARGET tbbmalloc_proxy_static APPEND_STRING PROPERTY COMPILE_FLAGS ${DISABLE_RTTI})
    if (TBB_INSTALL_TARGETS)
      install(TARGETS tbbmalloc_proxy_static ARCHIVE DESTINATION ${TBB_INSTALL_ARCHIVE_DIR})
    endif()
  endif()

  if (TBB_BUILD_SHARED)
    add_library(tbbmalloc_proxy SHARED ${tbbmalloc_proxy_src})
    target_link_libraries(tbbmalloc_proxy PRIVATE tbb_interface)
    set_property(TARGET tbbmalloc_proxy APPEND PROPERTY COMPILE_DEFINITIONS "__TBBMALLOC_BUILD=1")
    set_property(TARGET tbbmalloc_proxy APPEND_STRING PROPERTY COMPILE_FLAGS ${DISABLE_RTTI})
    if (TBB_SET_SOVERSION)
      set_property(TARGET tbbmalloc_proxy PROPERTY SOVERSION 2)
    endif ()
    target_link_libraries(tbbmalloc_proxy PUBLIC tbbmalloc)
    if (TBB_INSTALL_TARGETS)
      install(TARGETS tbbmalloc_proxy EXPORT TBB
              LIBRARY DESTINATION ${TBB_INSTALL_LIBRARY_DIR}
              ARCHIVE DESTINATION ${TBB_INSTALL_ARCHIVE_DIR}
              RUNTIME DESTINATION ${TBB_INSTALL_RUNTIME_DIR})
      if (MSVC)
        install(FILES $<TARGET_PDB_FILE:tbbmalloc_proxy> DESTINATION ${TBB_INSTALL_RUNTIME_DIR} OPTIONAL)
      endif()
    endif()
    if (UNIX AND NOT APPLE)
      target_link_libraries(tbbmalloc_proxy PUBLIC pthread dl)
    endif()
  endif()
endif()

if (TBB_INSTALL_TARGETS)
  install(DIRECTORY include/tbb DESTINATION ${TBB_INSTALL_INCLUDE_DIR})
  if (TBB_BUILD_SHARED)
    install(EXPORT TBB DESTINATION ${TBB_CMAKE_PACKAGE_INSTALL_DIR} NAMESPACE TBB:: FILE TBBConfig.cmake)
  endif()
endif()

# version file
if (TBB_INSTALL_TARGETS)
  set (_VERSION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/include/tbb/tbb_stddef.h)
  file (STRINGS ${_VERSION_FILE} _VERSION_MAJOR_STRING REGEX ".*define[ ]+TBB_VERSION_MAJOR[ ]+[0-9]+.*")
  file (STRINGS ${_VERSION_FILE} _VERSION_MINOR_STRING REGEX ".*define[ ]+TBB_VERSION_MINOR[ ]+[0-9]+.*")
  string (REGEX REPLACE ".*TBB_VERSION_MAJOR[ ]+([0-9]+)" "\\1" TBB_MAJOR_VERSION ${_VERSION_MAJOR_STRING})
  string (REGEX REPLACE ".*TBB_VERSION_MINOR[ ]+([0-9]+)" "\\1" TBB_MINOR_VERSION ${_VERSION_MINOR_STRING})
  set (TBB_VERSION_STRING "${TBB_MAJOR_VERSION}.${TBB_MINOR_VERSION}")
  include (CMakePackageConfigHelpers)
  write_basic_package_version_file (TBBConfigVersion.cmake VERSION "${TBB_VERSION_STRING}" COMPATIBILITY AnyNewerVersion)
  install (FILES ${CMAKE_CURRENT_BINARY_DIR}/TBBConfigVersion.cmake DESTINATION "${TBB_CMAKE_PACKAGE_INSTALL_DIR}")
endif()

# version_string.ver
if (UNIX AND NOT TBB_NO_DATE)
  execute_process (COMMAND date "+%a, %d %b %Y %H:%M:%S %z"
                   OUTPUT_VARIABLE _configure_date
                   OUTPUT_STRIP_TRAILING_WHITESPACE)
elseif (WIN32 AND NOT TBB_NO_DATE)
  execute_process (COMMAND cmd " /C date /T"
                   OUTPUT_VARIABLE _configure_date
                   OUTPUT_STRIP_TRAILING_WHITESPACE)
else ()
  set (_configure_date "Unknown")
endif()
set (TBB_CONFIG_DATE "${_configure_date}" CACHE STRING "First time that TBB was configured")
set (_configure_date "${TBB_CONFIG_DATE}")
include_directories (${CMAKE_BINARY_DIR})
#configure_file (build/version_string.ver.in version_string.ver @ONLY)

if (TBB_BUILD_TESTS)
  enable_language (C)
  enable_testing ()

  find_library (LIBRT_LIBRARIES rt)
  find_library (LIDL_LIBRARIES dl)
  find_package (Threads)
  if (NOT APPLE)
    find_package (OpenMP)
  endif()

  macro (tbb_add_test testname)
    set (full_testname tbb_test_${testname})
    add_executable (${full_testname} src/test/test_${testname}.cpp)
    target_link_libraries(${full_testname} PRIVATE tbb_interface)
    if (TBB_BUILD_SHARED)
      target_link_libraries (${full_testname} PRIVATE tbb tbbmalloc)
      target_compile_definitions (${full_testname} PRIVATE __TBB_LIB_NAME=tbb)
    else ()
      target_link_libraries (${full_testname} PRIVATE tbb_static tbbmalloc_static)
      target_compile_definitions (${full_testname} PRIVATE __TBB_LIB_NAME=tbb_static)
    endif ()
    if (LIBRT_LIBRARIES)
      target_link_libraries (${full_testname} PRIVATE ${LIBRT_LIBRARIES})
    endif ()
    if (LIDL_LIBRARIES)
      target_link_libraries (${full_testname} PRIVATE ${LIDL_LIBRARIES})
    endif ()
    if (Threads_FOUND)
      target_link_libraries (${full_testname} PRIVATE ${CMAKE_THREAD_LIBS_INIT})
    endif ()
    if (OPENMP_FOUND AND "${testname}" MATCHES "openmp")
      set_target_properties (${full_testname} PROPERTIES COMPILE_FLAGS "${OpenMP_CXX_FLAGS}")
      set_target_properties (${full_testname} PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")
    endif()
    if (MINGW)
      target_link_libraries (${full_testname} PRIVATE psapi)
    endif ()
    add_test (NAME ${full_testname} COMMAND ${full_testname})
  endmacro ()

  tbb_add_test (aggregator)
  tbb_add_test (aligned_space)
  tbb_add_test (assembly)
  if (NOT WIN32)
    tbb_add_test (async_msg) # msvc64/debug timeouts
  endif()
  tbb_add_test (async_node)
  # tbb_add_test (atomic) # msvc64/debug timeouts: Compile-time initialization fails for static tbb::atomic variables
  tbb_add_test (blocked_range2d)
  tbb_add_test (blocked_range3d)
  tbb_add_test (blocked_range)
  tbb_add_test (broadcast_node)
  tbb_add_test (buffer_node)
  tbb_add_test (cache_aligned_allocator)
  if (NOT WIN32)
    tbb_add_test (cache_aligned_allocator_STL)
  endif()
  tbb_add_test (cilk_dynamic_load)
  tbb_add_test (cilk_interop)
  tbb_add_test (combinable)
  tbb_add_test (composite_node)
  tbb_add_test (concurrent_hash_map)
  tbb_add_test (concurrent_lru_cache)
  # tbb_add_test (concurrent_monitor) # too long
  # tbb_add_test (concurrent_priority_queue)
  if (NOT WIN32)
    tbb_add_test (concurrent_queue) # msvc64/debug timeouts
  endif()
  # tbb_add_test (concurrent_queue_whitebox)
  tbb_add_test (concurrent_unordered_map)
  # tbb_add_test (concurrent_unordered_set)
  tbb_add_test (concurrent_vector)
  tbb_add_test (continue_node)
  tbb_add_test (critical_section)
  tbb_add_test (dynamic_link)
  # tbb_add_test (eh_algorithms)
  tbb_add_test (eh_flow_graph)
  # tbb_add_test (eh_tasks)
  tbb_add_test (enumerable_thread_specific)
  tbb_add_test (examples_common_utility)
  # tbb_add_test (fast_random)
  tbb_add_test (flow_graph)
  tbb_add_test (flow_graph_whitebox)
  # tbb_add_test (fp) # mingw: harness_fp.h:66, assertion !checkConsistency || (ctl.mxcsr & SSE_RND_MODE_MASK) >> 3 == (ctl.x87cw & FE_RND_MODE_MASK): failed
  # tbb_add_test (function_node) # mingw:random timeout
  # tbb_add_test (global_control)
  # tbb_add_test (global_control_whitebox)
  tbb_add_test (halt)
  tbb_add_test (handle_perror)
  # tbb_add_test (hw_concurrency)
  tbb_add_test (indexer_node)
  tbb_add_test (inits_loop)
  tbb_add_test (intrusive_list)
  tbb_add_test (ittnotify)
  # tbb_add_test (join_node) #msvc/64: fatal error C1128: number of sections exceeded object file format limit: compile with /bigob
  tbb_add_test (lambda)
  tbb_add_test (limiter_node)
  # tbb_add_test (malloc_atexit)
  # tbb_add_test (malloc_compliance) #mingw: Limits should be decreased for the test to work
  tbb_add_test (malloc_init_shutdown)
  # tbb_add_test (malloc_lib_unload)
  # tbb_add_test (malloc_overload)
  tbb_add_test (malloc_pools)
  tbb_add_test (malloc_regression)
  # tbb_add_test (malloc_used_by_lib)
  # tbb_add_test (malloc_whitebox)
  tbb_add_test (model_plugin)
  # tbb_add_test (multifunction_node) # too long
  tbb_add_test (mutex)
  tbb_add_test (mutex_native_threads)
  # tbb_add_test (opencl_node)
  if (OPENMP_FOUND)
    tbb_add_test (openmp)
  endif ()
  tbb_add_test (overwrite_node)
  # tbb_add_test (parallel_do)
  # This seems to fail on CI platforms (AppVeyor/Travis), perhaps because the VM exposes just 1 core?
  tbb_add_test (parallel_for)
  tbb_add_test (parallel_for_each)
  tbb_add_test (parallel_for_vectorization)
  tbb_add_test (parallel_invoke)
  tbb_add_test (parallel_pipeline)
  tbb_add_test (parallel_reduce)
  tbb_add_test (parallel_scan)
  tbb_add_test (parallel_sort)
  tbb_add_test (parallel_while)
  # tbb_add_test (partitioner_whitebox) # too long
  tbb_add_test (pipeline)
  # tbb_add_test (pipeline_with_tbf) # takes forever on appveyor
  tbb_add_test (priority_queue_node)
  tbb_add_test (queue_node)
  tbb_add_test (reader_writer_lock)
  # tbb_add_test (runtime_loader) # LINK : fatal error LNK1104: cannot open file 'tbbproxy.lib' [C:\projects\tbb\test_runtime_loader.vcxproj]
  tbb_add_test (rwm_upgrade_downgrade)
  # tbb_add_test (ScalableAllocator)
  if (NOT WIN32)
    tbb_add_test (ScalableAllocator_STL)
  endif()
  tbb_add_test (semaphore)
  # tbb_add_test (sequencer_node) # msvc: timeout
  tbb_add_test (source_node)
  tbb_add_test (split_node)
  tbb_add_test (static_assert)
  tbb_add_test (std_thread)
  tbb_add_test (tagged_msg)
  # tbb_add_test (task_arena) # LINK : fatal error LNK1104: cannot open file '__TBB_LIB_NAME.lib' [C:\projects\tbb\test_task_arena.vcxproj]
  # tbb_add_test (task_assertions)
  tbb_add_test (task_auto_init)
  tbb_add_test (task)
  # tbb_add_test (task_enqueue) # too long
  tbb_add_test (task_group)
  # tbb_add_test (task_leaks)
  # tbb_add_test (task_priority)
  # tbb_add_test (task_scheduler_init) # msvc: test_task_scheduler_init.cpp:68, assertion !test_mandatory_parallelism || Harness::CanReachConcurrencyLevel(threads): failed
  tbb_add_test (task_scheduler_observer)
  tbb_add_test (task_steal_limit)
  tbb_add_test (tbb_condition_variable)
  tbb_add_test (tbb_fork)
  # tbb_add_test (tbb_header)
  tbb_add_test (tbb_thread)
  # tbb_add_test (tbb_version)
  tbb_add_test (tick_count)
  tbb_add_test (tuple)
  tbb_add_test (write_once_node)
  tbb_add_test (yield)
endif ()

if (TBB_BUILD_PYTHON)
  find_package(PythonInterp)
  find_package(PythonLibs ${PYTHON_VERSION_STRING} EXACT)
  find_package(SWIG 3)
  if (PythonLibs_FOUND AND SWIG_FOUND AND TBB_BUILD_SHARED)
    include (${SWIG_USE_FILE})
    set_source_files_properties (python/tbb/api.i PROPERTIES CPLUSPLUS ON)
    set (CMAKE_SWIG_FLAGS "-threads")

    # swig_add_module is deprecated
    if (CMAKE_VERSION VERSION_LESS 3.8)
      swig_add_module (api python python/tbb/api.i)
    else ()
      swig_add_library (api LANGUAGE python SOURCES python/tbb/api.i)
    endif ()

    # UseSWIG generates now standard target names
    if (CMAKE_VERSION VERSION_LESS 3.13)
      set (module_target ${SWIG_MODULE_api_REAL_NAME})
    else ()
      set (module_target api)
    endif ()

    target_include_directories(${module_target} PRIVATE ${PYTHON_INCLUDE_DIRS})
    target_link_libraries(${module_target} PRIVATE tbb)
    if(WIN32)
      target_link_libraries(${module_target} ${PYTHON_LIBRARIES})
    elseif(APPLE)
      set_target_properties(${module_target} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
    endif()

    if (WIN32)
      set (PYTHON_SITE_PACKAGES Lib/site-packages)
    else ()
      set (PYTHON_SITE_PACKAGES lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages)
    endif ()
    if (TBB_INSTALL_TARGETS)
      install(FILES python/TBB.py
              DESTINATION ${PYTHON_SITE_PACKAGES})
      install(FILES python/tbb/__init__.py python/tbb/pool.py python/tbb/test.py python/tbb/__main__.py ${CMAKE_CURRENT_BINARY_DIR}/api.py
              DESTINATION ${PYTHON_SITE_PACKAGES}/tbb)
      install(TARGETS ${module_target}
              DESTINATION ${PYTHON_SITE_PACKAGES}/tbb)
    endif()

    if(UNIX AND NOT APPLE)
      add_library(irml SHARED python/rml/ipc_server.cpp python/rml/ipc_utils.cpp src/tbb/cache_aligned_allocator.cpp src/tbb/dynamic_link.cpp src/tbb/tbb_misc_ex.cpp src/tbb/tbb_misc.cpp)
      target_compile_definitions(irml PRIVATE DO_ITT_NOTIFY=0 USE_PTHREAD=1)
      target_link_libraries(irml PRIVATE tbb)
      set_target_properties(irml PROPERTIES VERSION 1)
      if (TBB_INSTALL_TARGETS)
        install(TARGETS irml DESTINATION ${TBB_INSTALL_LIBRARY_DIR})
      endif()
    endif ()
  endif ()
endif ()
