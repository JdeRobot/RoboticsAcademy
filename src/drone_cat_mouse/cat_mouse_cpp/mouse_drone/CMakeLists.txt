cmake_minimum_required(VERSION 2.8)
project(mouse_drone)

include(FindPkgConfig)

SET(SOURCE_FILES mouse.cpp travel.cpp parser.cpp
                quaternion.cpp
                config/drone_config.cpp config/path_config.cpp
                config/travel_config.cpp config/mouse_config.cpp
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

include_directories (
    ${JderobotInterfaces_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/config
)

PKG_CHECK_MODULES(libxmlpp REQUIRED libxml++-2.6)
include_directories(${libxmlpp_INCLUDE_DIRS})
link_directories(
	${libxmlpp_LIBRARY_DIRS} 	
	${JderobotInterfaces_LIBRARY_DIRS}
)

add_executable(mousedrone
   ${SOURCE_FILES}
)

target_link_libraries(mousedrone
    Ice
    IceUtil
	pthread
	${libxmlpp_LIBRARIES}
	${JderobotInterfaces_LIBRARIES}
)
