find_package(TBB REQUIRED)
#find_package(pagmo QUIET)

if (NOT TBB_FOUND)
    option(TBB_BUILD_STATIC          "Build TBB static library" OFF)
    option(TBB_BUILD_SHARED          "Build TBB shared library" ON)
    option(TBB_INSTALL_TARGETS       "Include build targets for 'make install'" OFF)
#    add_subdirectory(tbb)
endif ()

if (NOT pagmo_FOUND)
    option(PAGMO_WITH_EIGEN3 "Enable features depending on Eigen3 (such as CMAES). Requires Eigen3." ON)
    option(PAGMO_BUILD_STATIC_LIBRARY "Build pagmo as a static library, instead of dynamic." OFF)
    add_subdirectory(pagmo2)
endif ()

add_subdirectory(hpp-fcl)