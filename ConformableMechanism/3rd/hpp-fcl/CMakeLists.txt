#
# Software License Agreement (BSD License)
#
#  Copyright (c) 2014-2023 CNRS-LAAS, INRIA
#  Author: Florent Lamiraux, Joseph Mirabel
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of CNRS-LAAS. nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.10)
set(CXX_DISABLE_WERROR TRUE)

set(PROJECT_NAME hpp-fcl)
set(PROJECT_DESCRIPTION
  "HPP fork of FCL -- The Flexible Collision Library"
  )
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(PROJECT_COMPATIBILITY_VERSION AnyNewerVersion)

SET(PROJECT_USE_KEYWORD_LINK_LIBRARIES TRUE)
SET(DOXYGEN_USE_TEMPLATE_CSS TRUE)

# ----------------------------------------------------
# --- OPTIONS  ---------------------------------------
# Need to be set before including base.cmake
# ----------------------------------------------------
option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)
option(HPP_FCL_TURN_ASSERT_INTO_EXCEPTION "Turn some critical HPP-FCL asserts to exception." FALSE)
option(HPP_FCL_ENABLE_LOGGING "Activate logging for warnings or error messages. Turned on by default in Debug." FALSE)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(EXISTS "${JRL_CMAKE_MODULES}/base.cmake")
  message(STATUS "JRL cmakemodules found in 'cmake/' git submodule")
else()
  find_package(jrl-cmakemodules QUIET CONFIG)
  if(jrl-cmakemodules_FOUND)
    get_property(
      JRL_CMAKE_MODULES
      TARGET jrl-cmakemodules::jrl-cmakemodules
      PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}")
  elseif(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    message(
      FATAL_ERROR
        "\nCan't find jrl-cmakemodules. Please either:\n"
        "  - use git submodule: 'git submodule update --init'\n"
        "  - or install https://github.com/jrl-umi3218/jrl-cmakemodules\n"
        "  - or upgrade your CMake version to >= 3.14 to allow automatic fetching\n"
    )
  else()
    message(STATUS "JRL cmakemodules not found. Let's fetch it.")
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

include("${JRL_CMAKE_MODULES}/boost.cmake")
include("${JRL_CMAKE_MODULES}/hpp.cmake")
include("${JRL_CMAKE_MODULES}/apple.cmake")
include("${JRL_CMAKE_MODULES}/ide.cmake")
include(CMakeDependentOption)

SET(CMAKE_MODULE_PATH
  ${JRL_CMAKE_MODULES}/find-external/assimp/
  ${CMAKE_MODULE_PATH})

set_default_cmake_build_type("RelWithDebInfo")

# If needed, fix CMake policy for APPLE systems
APPLY_DEFAULT_APPLE_CONFIGURATION()

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

OPTION(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
CMAKE_DEPENDENT_OPTION(GENERATE_PYTHON_STUBS "Generate the Python stubs associated to the Python library" OFF BUILD_PYTHON_INTERFACE OFF)

#IF(WIN32)
#  # Set default Windows build paths
#  SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY
#    ${PROJECT_BINARY_DIR}/Bin
#    CACHE PATH "Single directory for all libraries")
#  SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY
#    ${PROJECT_BINARY_DIR}/Bin
#    CACHE PATH "Single directory for all executables")
#  SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
#    ${PROJECT_BINARY_DIR}/Bin
#    CACHE PATH "Sing$le directory for all archives")
#ENDIF(WIN32)

if (NOT TARGET Eigen3::Eigen)
  ADD_PROJECT_DEPENDENCY(Eigen3 REQUIRED PKG_CONFIG_REQUIRES "eigen3 >= 3.0.0")
endif ()

# Required dependencies
SET_BOOST_DEFAULT_OPTIONS()
EXPORT_BOOST_DEFAULT_OPTIONS()


if(Boost_VERSION_STRING VERSION_LESS 1.81)
  # Default C++ version should be C++11
  CHECK_MINIMAL_CXX_STANDARD(11 ENFORCE)
else()
  # Boost.Math will be C++14 starting in July 2023 (Boost 1.82 release)
  CHECK_MINIMAL_CXX_STANDARD(14 ENFORCE)
endif()

# Optional dependencies
ADD_PROJECT_DEPENDENCY(octomap PKG_CONFIG_REQUIRES "octomap >= 1.6")
if(octomap_FOUND)
  SET(HPP_FCL_HAS_OCTOMAP TRUE)
  string(REPLACE "." ";" VERSION_LIST ${octomap_VERSION})
  list(GET VERSION_LIST 0 OCTOMAP_MAJOR_VERSION)
  list(GET VERSION_LIST 1 OCTOMAP_MINOR_VERSION)
  list(GET VERSION_LIST 2 OCTOMAP_PATCH_VERSION)
  message(STATUS "HPP-FCL uses Octomap")
else()
  SET(HPP_FCL_HAS_OCTOMAP FALSE)
  message(STATUS "HPP-FCL does not use Octomap")
endif()

option(HPP_FCL_HAS_QHULL "use qhull library to compute convex hulls." FALSE)
if(HPP_FCL_HAS_QHULL)
  find_package(Qhull COMPONENTS qhull_r qhullcpp)
  if(Qhull_FOUND)
    set(HPP_FCL_USE_SYSTEM_QHULL TRUE)
    message(STATUS "HPP-FCL uses system Qhull")
  else()
    message(STATUS "Qhullcpp not found: it will be build from sources, if Qhull_r is found")
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/third-parties)
    execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
      ${CMAKE_SOURCE_DIR}/third-parties/qhull/src/libqhullcpp
      ${CMAKE_CURRENT_BINARY_DIR}/third-parties/libqhullcpp
      )
    set(Qhullcpp_PREFIX ${CMAKE_BINARY_DIR}/third-parties)
    find_path(Qhull_r_INCLUDE_DIR
      NAMES libqhull_r/libqhull_r.h
      PATHS ${Qhull_PREFIX}
      )
    find_library(Qhull_r_LIBRARY
      NAMES libqhull_r.so
      PATHS ${Qhull_PREFIX}
      )
    if(NOT Qhull_r_LIBRARY)
      message(FATAL_ERROR "Qhull_r not found, please install it or turn HPP_FCL_HAS_QHULL OFF")
    endif()
  endif()
endif()

SET(${PROJECT_NAME}_HEADERS
  include/hpp/fcl/collision_data.h
  include/hpp/fcl/BV/kIOS.h
  include/hpp/fcl/BV/BV.h
  include/hpp/fcl/BV/RSS.h
  include/hpp/fcl/BV/OBBRSS.h
  include/hpp/fcl/BV/BV_node.h
  include/hpp/fcl/BV/AABB.h
  include/hpp/fcl/BV/OBB.h
  include/hpp/fcl/BV/kDOP.h
  include/hpp/fcl/broadphase/broadphase.h
  include/hpp/fcl/broadphase/broadphase_SSaP.h
  include/hpp/fcl/broadphase/broadphase_SaP.h
  include/hpp/fcl/broadphase/broadphase_bruteforce.h
  include/hpp/fcl/broadphase/broadphase_collision_manager.h
  include/hpp/fcl/broadphase/broadphase_continuous_collision_manager-inl.h
  include/hpp/fcl/broadphase/broadphase_continuous_collision_manager.h
  include/hpp/fcl/broadphase/broadphase_dynamic_AABB_tree-inl.h
  include/hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h
  include/hpp/fcl/broadphase/broadphase_dynamic_AABB_tree_array-inl.h
  include/hpp/fcl/broadphase/broadphase_dynamic_AABB_tree_array.h
  include/hpp/fcl/broadphase/broadphase_interval_tree.h
  include/hpp/fcl/broadphase/broadphase_spatialhash-inl.h
  include/hpp/fcl/broadphase/broadphase_spatialhash.h
  include/hpp/fcl/broadphase/broadphase_callbacks.h
  include/hpp/fcl/broadphase/default_broadphase_callbacks.h
  include/hpp/fcl/broadphase/detail/hierarchy_tree-inl.h
  include/hpp/fcl/broadphase/detail/hierarchy_tree.h
  include/hpp/fcl/broadphase/detail/hierarchy_tree_array-inl.h
  include/hpp/fcl/broadphase/detail/hierarchy_tree_array.h
  include/hpp/fcl/broadphase/detail/interval_tree.h
  include/hpp/fcl/broadphase/detail/interval_tree_node.h
  include/hpp/fcl/broadphase/detail/morton-inl.h
  include/hpp/fcl/broadphase/detail/morton.h
  include/hpp/fcl/broadphase/detail/node_base-inl.h
  include/hpp/fcl/broadphase/detail/node_base.h
  include/hpp/fcl/broadphase/detail/node_base_array-inl.h
  include/hpp/fcl/broadphase/detail/node_base_array.h
  include/hpp/fcl/broadphase/detail/simple_hash_table-inl.h
  include/hpp/fcl/broadphase/detail/simple_hash_table.h
  include/hpp/fcl/broadphase/detail/simple_interval-inl.h
  include/hpp/fcl/broadphase/detail/simple_interval.h
  include/hpp/fcl/broadphase/detail/sparse_hash_table-inl.h
  include/hpp/fcl/broadphase/detail/sparse_hash_table.h
  include/hpp/fcl/broadphase/detail/spatial_hash-inl.h
  include/hpp/fcl/broadphase/detail/spatial_hash.h
  include/hpp/fcl/narrowphase/narrowphase.h
  include/hpp/fcl/narrowphase/gjk.h
  include/hpp/fcl/narrowphase/narrowphase_defaults.h
  include/hpp/fcl/narrowphase/minkowski_difference.h
  include/hpp/fcl/narrowphase/support_functions.h
  include/hpp/fcl/shape/convex.h
  include/hpp/fcl/shape/details/convex.hxx
  include/hpp/fcl/shape/geometric_shape_to_BVH_model.h
  include/hpp/fcl/shape/geometric_shapes.h
  include/hpp/fcl/shape/geometric_shapes_traits.h
  include/hpp/fcl/shape/geometric_shapes_utility.h
  include/hpp/fcl/distance_func_matrix.h
  include/hpp/fcl/collision.h
  include/hpp/fcl/collision_func_matrix.h
  include/hpp/fcl/contact_patch.h
  include/hpp/fcl/contact_patch_func_matrix.h
  include/hpp/fcl/contact_patch/contact_patch_solver.h
  include/hpp/fcl/contact_patch/contact_patch_solver.hxx
  include/hpp/fcl/distance.h
  include/hpp/fcl/math/matrix_3f.h
  include/hpp/fcl/math/vec_3f.h
  include/hpp/fcl/math/types.h
  include/hpp/fcl/math/transform.h
  include/hpp/fcl/data_types.h
  include/hpp/fcl/BVH/BVH_internal.h
  include/hpp/fcl/BVH/BVH_model.h
  include/hpp/fcl/BVH/BVH_front.h
  include/hpp/fcl/BVH/BVH_utility.h
  include/hpp/fcl/collision_object.h
  include/hpp/fcl/collision_utility.h
  include/hpp/fcl/hfield.h
  include/hpp/fcl/fwd.hh
  include/hpp/fcl/logging.h
  include/hpp/fcl/mesh_loader/assimp.h
  include/hpp/fcl/mesh_loader/loader.h
  include/hpp/fcl/internal/BV_fitter.h
  include/hpp/fcl/internal/BV_splitter.h
  include/hpp/fcl/internal/shape_shape_func.h
  include/hpp/fcl/internal/shape_shape_contact_patch_func.h
  include/hpp/fcl/internal/intersect.h
  include/hpp/fcl/internal/tools.h
  include/hpp/fcl/internal/traversal_node_base.h
  include/hpp/fcl/internal/traversal_node_bvh_shape.h
  include/hpp/fcl/internal/traversal_node_bvhs.h
  include/hpp/fcl/internal/traversal_node_hfield_shape.h
  include/hpp/fcl/internal/traversal_node_setup.h
  include/hpp/fcl/internal/traversal_node_shapes.h
  include/hpp/fcl/internal/traversal_recurse.h
  include/hpp/fcl/internal/traversal.h
  include/hpp/fcl/serialization/fwd.h
  include/hpp/fcl/serialization/serializer.h
  include/hpp/fcl/serialization/archive.h
  include/hpp/fcl/serialization/transform.h
  include/hpp/fcl/serialization/AABB.h
  include/hpp/fcl/serialization/BV_node.h
  include/hpp/fcl/serialization/BV_splitter.h
  include/hpp/fcl/serialization/BVH_model.h
  include/hpp/fcl/serialization/collision_data.h
  include/hpp/fcl/serialization/contact_patch.h
  include/hpp/fcl/serialization/collision_object.h
  include/hpp/fcl/serialization/convex.h
  include/hpp/fcl/serialization/eigen.h
  include/hpp/fcl/serialization/geometric_shapes.h
  include/hpp/fcl/serialization/memory.h
  include/hpp/fcl/serialization/OBB.h
  include/hpp/fcl/serialization/RSS.h
  include/hpp/fcl/serialization/OBBRSS.h
  include/hpp/fcl/serialization/kIOS.h
  include/hpp/fcl/serialization/kDOP.h
  include/hpp/fcl/serialization/hfield.h
  include/hpp/fcl/serialization/quadrilateral.h
  include/hpp/fcl/serialization/triangle.h
  include/hpp/fcl/timings.h
  )

IF(HPP_FCL_HAS_OCTOMAP)
  LIST(APPEND ${PROJECT_NAME}_HEADERS
    include/hpp/fcl/octree.h
    include/hpp/fcl/serialization/octree.h
    include/hpp/fcl/internal/traversal_node_octree.h
  )
ENDIF(HPP_FCL_HAS_OCTOMAP)
set(IncludeDir "${CMAKE_CURRENT_SOURCE_DIR}/include")
add_subdirectory(src)

pkg_config_append_libs("hpp-fcl")
IF(HPP_FCL_HAS_OCTOMAP)
  # FCL_HAVE_OCTOMAP kept for backward compatibility reasons.
  PKG_CONFIG_APPEND_CFLAGS(
    "-DHPP_FCL_HAS_OCTOMAP -DHPP_FCL_HAVE_OCTOMAP -DFCL_HAVE_OCTOMAP -DOCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION} -DOCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION} -DOCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION}")
ENDIF(HPP_FCL_HAS_OCTOMAP)

# Install catkin package.xml
INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})
# Allows Colcon to find non-Ament packages when using workspace underlays
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv "prepend-non-duplicate;AMENT_PREFIX_PATH;")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv DESTINATION share/${PROJECT_NAME}/hook)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv "prepend-non-duplicate;PYTHONPATH;${PYTHON_SITELIB}")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv DESTINATION share/${PROJECT_NAME}/hook)
