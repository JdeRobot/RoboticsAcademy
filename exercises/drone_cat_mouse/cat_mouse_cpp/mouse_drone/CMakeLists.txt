cmake_minimum_required(VERSION 2.8)
project(mouse_drone C CXX)


#check compiller
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	include(CheckCXXCompilerFlag)
	CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
	CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
	message (version: ${CMAKE_CXX_COMPILER_VERSION})
	if(COMPILER_SUPPORTS_CXX11)
		set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++11)
	elseif(COMPILER_SUPPORTS_CXX0X)
		#boost have problems with gcc < 4.7 and c++0x
		if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "4.7.0")
			set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
		endif()
	else()
		message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
	endif()
endif()

IF (NOT CMAKE_BUILD_TYPE)
	MESSAGE("SETTING BUILD TYPE TO RELEASE")
	SET(CMAKE_BUILD_TYPE RELEASE)
ENDIF()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -fPIC")

include(FindPkgConfig)

FIND_PACKAGE(JdeRobot REQUIRED)
include(Deps/ice/CMakeLists.txt)


SET(SOURCE_FILES mouse.cpp travel.cpp parser.cpp
                quaternion.cpp
                config/drone_config.cpp config/path_config.cpp
                config/travel_config.cpp config/mouse_config.cpp
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

include_directories (
    ${JdeRobot_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/config
)

PKG_CHECK_MODULES(libxmlpp REQUIRED libxml++-2.6)
include_directories(${libxmlpp_INCLUDE_DIRS})
link_directories(
	${libxmlpp_LIBRARY_DIRS} 	
	${JdeRobot_INCLUDE_DIRS}
)

add_executable(mousedrone
   ${SOURCE_FILES}
)

target_link_libraries(mousedrone
	${ZeroCIce_LIBRARIES}
	${libxmlpp_LIBRARIES}
	${JdeRobot_LIBRARIES}
	pthread
)
