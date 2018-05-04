SET(CMAKE_MODULE_PATH
	${CMAKE_MODULE_PATH}
	"${CMAKE_CURRENT_LIST_DIR}/CMake"
)

SET(slice_path /usr/share/slice)

#ICE installs c++11 libraries in this directory and we don't indicate this, cmake uses c++98 files
SET(CMAKE_PREFIX_PATH /usr/lib/x86_64-linux-gnu/c++11)
FIND_PACKAGE(ZeroCIceUtil)
FIND_PACKAGE(ZeroCIceStorm)
FIND_PACKAGE(ZeroCIce)
	IF( ZeroCIceStorm_LIBRARY )
		IF( ZeroCIceUtil_LIBRARY )
			MESSAGE("*** ICE ok")
			include_directories(${ZeroCIceUtil_INCLUDE_DIR})
			link_directories(${ZeroCIceUtil_LIBRARY})
			include_directories(${ZeroCIceCore_INCLUDE_DIR})
			link_directories(${ZeroCIceCore_LIBRARY})
			include_directories(${ZeroCIceStorm_INCLUDE_DIR})
			link_directories(${ZeroCIceStorm_LIBRARY})
			include_directories(${ZeroCIce_INCLUDE_DIR})
			link_directories(${ZeroCIce_LIBRARIES})

			list(APPEND DEPS libzeroc-ice3.6 zeroc-ice-utils libzeroc-icestorm3.6)
			list(APPEND DEPS_DEV zeroc-ice-slice libzeroc-ice-dev)
			

		ENDIF( ZeroCIceUtil_LIBRARY )
	ENDIF( ZeroCIceStorm_LIBRARY )

IF(NOT  ZeroCIceStorm_LIBRARY )
	MESSAGE(FATAL_ERROR "*** IceStorm not found")
ENDIF()

IF(NOT  ZeroCIceUtil_LIBRARY )
	MESSAGE(FATAL_ERROR "*** IceUtil not found")
ENDIF()

