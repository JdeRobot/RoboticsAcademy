# Copyright (C) 2015 JDE Developers Team
# Authors:
#   2015 Dec, Victor Arribas <v.arribas.urjc@gmail.com>

project (jderobot-teachingrobotics)
cmake_minimum_required(VERSION 2.8)


## Tools
include(cmake/Tools/check_compiler.cmake)
include(cmake/Tools/find_packages.cmake)
include(cmake/Tools/declare_package.cmake)
include(cmake/Tools/use_package.cmake)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/Tools/CMake")


## Dependencies
# allow Deps to populate
#  * include_directories
#  * link_directories
# for JdeRobot libraries
set(INJECT_JDE_LIBS ON)
# for 3rd party libraries
set(INJECT_DEPS_LIBS OFF)

# alphabetical sorted to handle dependency order
# this means that you must to prepend letter based sort at directory level
file(GLOB_RECURSE CMakeLists_files "cmake/Deps/**/CMakeLists.txt")
list(SORT CMakeLists_files)
foreach(CMakeLists_file ${CMakeLists_files})
    message(STATUS "${CMakeLists_file}")
    include(${CMakeLists_file})
endforeach()


## Code (user code)
# alphabetical sorted to handle dependency order
file(GLOB_RECURSE CMakeLists_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/**/CMakeLists.txt")
list(SORT CMakeLists_files)
if (CMakeLists_files)
    string(REGEX REPLACE "//" "/" list ${CMakeLists_files})
    string(REGEX REPLACE "/CMakeLists.txt" "" list ${list})
    string(REGEX REPLACE "src/" "\n\t- " list ${list})
    message(STATUS "Found following components to build: ${list}\n")

    foreach(CMakeLists_file ${CMakeLists_files})
        get_filename_component(CMakeLists_dir ${CMakeLists_file} PATH)
        string(REGEX REPLACE "src/" "" name ${CMakeLists_dir})
        message(STATUS "Configuring '${name}'")
        add_subdirectory(${CMakeLists_dir})
    endforeach()
endif()

#############
## INSTALL ##
#############

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/teaching_robotics)
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/teaching_robotics)
INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/teaching_robotics)

#############
## DEB PKG ##
#############

# Determine current architecture
macro(dpkg_arch VAR_NAME)
        find_program(DPKG_PROGRAM dpkg DOC "dpkg program of Debian-based systems")
        if (DPKG_PROGRAM) 
          execute_process(
            COMMAND ${DPKG_PROGRAM} --print-architecture
            OUTPUT_VARIABLE ${VAR_NAME}
            OUTPUT_STRIP_TRAILING_WHITESPACE
          )
        endif(DPKG_PROGRAM)
endmacro(dpkg_arch)

include (InstallRequiredSystemLibraries)
SET (CPACK_GENERATOR "DEB")
SET (CPACK_SOURCE_GENERATOR TGZ)
SET (CPACK_SET_DESTDIR ON)
SET (CPACK_DEB_COMPONENT_INSTALL OFF)
SET (CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
SET (CPACK_PACKAGING_INSTALL_PREFIX "/usr/local")


# CPack version numbers for release tarball name.
SET (CPACK_PACKAGE_VERSION_MAJOR 0)
SET (CPACK_PACKAGE_VERSION_MINOR 1)
SET (CPACK_PACKAGE_VERSION_PATCH 4)
SET (CPACK_DEBIAN_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH})

## Inject release roll up (incremental releases)
set(PROJECT_VERSION ${CPACK_DEBIAN_PACKAGE_VERSION})
set(CPACK_DEBIAN_PACKAGE_VERSION ${PROJECT_VERSION})


SET (CPACK_DEBIAN_PACKAGE_PRIORITY "extra")
SET (CPACK_DEBIAN_PACKAGE_SECTION "net")
dpkg_arch(CPACK_DEBIAN_PACKAGE_ARCHITECTURE)

SET (CPACK_MONOLITHIC_INSTALL ON)
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "git, cmake, libgtkgl2.0-dev, libgtkmm-2.4-dev, libglademm-2.4-dev, libgnomecanvasmm-2.6-dev, libgtkglextmm-x11-1.2-dev, libzeroc-ice35-dev, libboost-system1.54-dev, libboost-filesystem1.54-dev, libtinyxml-dev, libssl-dev, freeglut3-dev, libglu1-mesa-dev, libgsl0-dev, libxml++2.6-dev, libopencv-core-dev, libopencv-flann-dev, libopencv-imgproc-dev, libopencv-ml-dev, libopencv-features2d-dev, libopencv-video-dev, libopencv-objdetect-dev, libopencv-contrib-dev, libopencv-gpu-dev, libopencv-legacy-dev, libopencv-dev, python-gnome2, python-docutils, jderobot")

SET (CPACK_PACKAGE_DESCRIPTION_SUMMARY
"JdeRobot is a software development suite for robotics applications.")
SET (CPACK_PACKAGE_DESCRIPTION
"JdeRobot is a software development suite for robotics applications,
 written in C++ language.
 It provides a programming environment where the robot control program is made
 up of a collection of several concurrent asynchronous threads named schemas.
 It is based on JDE cognitive architecture for autonomous robots.
 .
 Get source from https://github.com/RoboticsURJC/JdeRobot
 Package created with revision ${GIT_HEAD}")

## Patch: CPACK_PACKAGE_DESCRIPTION behavior is broken. Always use SUMMARY
SET (CPACK_PACKAGE_DESCRIPTION_SUMMARY ${CPACK_PACKAGE_DESCRIPTION})

SET (CPACK_PACKAGE_CONTACT "Roberto Calvo <rocapal@gsyc.urjc.es>")
SET (CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}_${CPACK_DEBIAN_PACKAGE_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

SET (PACKAGE_VERSION ${CPACK_DEBIAN_PACKAGE_VERSION})

#SET (CPACK_COMPONENTS_ALL Libraries ApplicationData)
include (CPack Documentation)
