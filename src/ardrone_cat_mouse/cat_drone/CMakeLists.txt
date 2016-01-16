cmake_minimum_required(VERSION 2.8)
include(FindPkgConfig)

SET(SOURCE_FILES cat.cpp
                drone_sensors.cpp
                object_detected.cpp
                quaternion.cpp
                cat_config.cpp
                parser.cpp
)

SET (INTERFACES_CPP_DIR ~/GitJdeRobot/JdeRobot/src/stable/interfaces/cpp/jderobot)
SET (LIBS_DIR ~/GitJdeRobot/JdeRobot/src/stable/libs)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

include_directories (
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

link_directories(${INTERFACES_CPP_DIR})

PKG_CHECK_MODULES(libxmlpp REQUIRED libxml++-2.6)
include_directories(${libxmlpp_INCLUDE_DIRS})
link_directories(${libxmlpp_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)

add_executable(catdrone
    ${SOURCE_FILES}
)

target_link_libraries(catdrone
	${OpenCV_LIBS}
    Ice
    IceUtil
	${libxmlpp_LIBRARIES}
	JderobotInterfaces
	#/usr/local/lib/jderobot/libjderobotice.so
	#/usr/local/lib/jderobot/libjderobotutil.so
)
