cmake_minimum_required(VERSION 2.8)
project(cat_drone)

include(FindPkgConfig)

SET(SOURCE_FILES cat.cpp
                drone_sensors.cpp
                object_detected.cpp
                quaternion.cpp
                cat_config.cpp
                parser.cpp
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

include_directories (
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${JderobotInterfaces_INCLUDE_DIRS}

)

PKG_CHECK_MODULES(libxmlpp REQUIRED libxml++-2.6)
include_directories(${libxmlpp_INCLUDE_DIRS})
link_directories(
	${libxmlpp_LIBRARY_DIRS}
	${JderobotInterfaces_LIBRARY_DIRS}
)

find_package(OpenCV REQUIRED)

add_executable(catdrone
    ${SOURCE_FILES}
)

target_link_libraries(catdrone
	${OpenCV_LIBS}
    Ice
    IceUtil
	pthread
	${libxmlpp_LIBRARIES}
	${JderobotInterfaces_LIBRARIES}
)
