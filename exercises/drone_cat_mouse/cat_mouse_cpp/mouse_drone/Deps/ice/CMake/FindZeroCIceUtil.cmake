# Find the ZeroC ICEUtil includes and libraries

#
# ZeroCIceUtil_INCLUDE_DIR
# ZeroCIceUtil_LIBRARIES
# ZerocCIceCore_FOUND


#
# Copyright (c) 2007, Pau Garcia i Quiles, <pgquiles@elpauer.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

FIND_PATH( ZeroCIceUtil_INCLUDE_DIR NAMES IceUtil/IceUtil.h PATHS ENV C++LIB ENV PATH PATH_SUFFIXES include Ice Ice/include )

IF( ZeroCIceUtil_INCLUDE_DIR )
	FIND_LIBRARY( ZeroCIceUtil_LIBRARY NAMES IceUtil PATHS ENV C++LIB ENV PATH PATH_SUFFIXES Ice lib-release lib_release )

	IF( ZeroCIceUtil_LIBRARY )
		SET( ZeroCIceUtil_FOUND TRUE )
	ENDIF( ZeroCIceUtil_LIBRARY )


ENDIF( ZeroCIceUtil_INCLUDE_DIR )
