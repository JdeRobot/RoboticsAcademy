# Find the ZeroC ICEStorm includes and libraries

#
# ZeroCIceStorm_INCLUDE_DIR
# ZeroCIceStorm_LIBRARIES
# ZerocCIceStorm_FOUND


#
# Copyright (c) 2007, Pau Garcia i Quiles, <pgquiles@elpauer.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

FIND_PATH( ZeroCIceStorm_INCLUDE_DIR NAMES IceStorm/IceStorm.h PATHS ENV C++LIB ENV PATH PATH_SUFFIXES include Ice Ice/include )

IF( ZeroCIceStorm_INCLUDE_DIR )
	FIND_LIBRARY( ZeroCIceStorm_LIBRARY NAMES IceStorm PATHS ENV C++LIB ENV PATH PATH_SUFFIXES Ice lib-release lib_release )

	IF( ZeroCIceStorm_LIBRARY )
		SET( ZeroCIceStorm_FOUND TRUE )
	ENDIF( ZeroCIceStorm_LIBRARY )


ENDIF( ZeroCIceStorm_INCLUDE_DIR )
