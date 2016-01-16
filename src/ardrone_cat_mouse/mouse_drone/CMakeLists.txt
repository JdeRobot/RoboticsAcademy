cmake_minimum_required(VERSION 2.8)
include(FindPkgConfig)

SET(SOURCE_FILES mouse.cpp travel.cpp parser.cpp
                quaternion.cpp
                config/drone_config.cpp config/path_config.cpp
                config/travel_config.cpp config/mouse_config.cpp
                #drone_sensors.cpp
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

SET (INTERFACES_CPP_DIR /usr/local/lib/jderobot)
SET (LIBS_DIR /usr/local/include/jderobot)
SET (MY_ICE_INTERFACES_DIR ~/pfc/trunk/src/interfaces/cpp)

include_directories (
#    /usr/include/eigen3
 #   /usr/local/lib/jderobot/progeo
    ${MY_ICE_INTERFACES_DIR}
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/config
)

PKG_CHECK_MODULES(libxmlpp REQUIRED libxml++-2.6)
include_directories(${libxmlpp_INCLUDE_DIRS})
link_directories(${libxmlpp_LIBRARY_DIRS})

add_executable(mousedrone
    ${MY_ICE_INTERFACES_DIR}/ardroneextra.cpp
    ${MY_ICE_INTERFACES_DIR}/cmdvel.cpp
    ${MY_ICE_INTERFACES_DIR}/remoteConfig.cpp
    ${SOURCE_FILES}
)

target_link_libraries(mousedrone
    Ice
    IceUtil
	${libxmlpp_LIBRARIES}
	/usr/local/lib/jderobot/libJderobotInterfaces.so
	#/usr/local/lib/jderobot/libjderobotice.so
	/usr/local/lib/jderobot/libjderobotutil.so
)
