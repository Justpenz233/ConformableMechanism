if(TARGET CGAL::CGAL)
    return()
endif()

message(STATUS "Third-party: creating target 'CGAL::CGAL'")

cmake_policy(SET CMP0135 NEW)
include(FetchContent)
FetchContent_Declare(
    cgal
    URL https://github.com/CGAL/cgal/releases/download/v6.0.1/CGAL-6.0.1-library.tar.xz
)
FetchContent_GetProperties(cgal)
if(cgal_POPULATED)
    return()
endif()
FetchContent_Populate(cgal)

function(cgal_import_target)
    macro(ignore_package NAME VERSION_NUM)
        include(CMakePackageConfigHelpers)
        file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/${NAME}Config.cmake "")
        write_basic_package_version_file(
            ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/${NAME}ConfigVersion.cmake
            VERSION ${VERSION_NUM}
            COMPATIBILITY AnyNewerVersion
        )
        set(${NAME}_DIR ${CMAKE_CURRENT_BINARY_DIR}/${NAME} CACHE PATH "")
        set(${NAME}_ROOT ${CMAKE_CURRENT_BINARY_DIR}/${NAME} CACHE PATH "")
    endmacro()

    include(boost)

    ignore_package(Boost 1.81.0)
    set(Boost_INCLUDE_DIRS "")
    set(Boost_LIBRARIES Boost::thread Boost::system Boost::any Boost::variant Boost::foreach Boost::math Boost::graph Boost::property_map Boost::multiprecision Boost::heap)

    # Prefer Config mode before Module mode to prevent CGAL from loading its own FindXXX.cmake
    set(CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE)

    # https://stackoverflow.com/a/71714947/148668
    set(CGAL_DATA_DIR "unspecified")
    find_package(CGAL CONFIG COMPONENTS Core PATHS ${cgal_SOURCE_DIR} NO_DEFAULT_PATH)
endfunction()

cgal_import_target()
