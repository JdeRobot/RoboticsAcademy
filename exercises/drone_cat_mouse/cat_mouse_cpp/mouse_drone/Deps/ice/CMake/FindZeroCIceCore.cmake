# Find the ZeroC ICE essential includes and libraries

#
# ZeroCIceCore_INCLUDE_DIR
# ZeroCIceCore_LIBRARIES
# ZeroCIceCore_FOUND


#
# Copyright (c) 2007, Pau Garcia i Quiles, <pgquiles@elpauer.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

FIND_PATH( ZeroCIceCore_INCLUDE_DIR NAMES Ice/Ice.h PATHS ENV C++LIB ENV PATH PATH_SUFFIXES include Ice Ice/include )

IF( ZeroCIceCore_INCLUDE_DIR )
	FIND_LIBRARY( ZeroCIceCore_LIBRARY NAMES Ice PATHS ENV C++LIB ENV PATH PATH_SUFFIXES Ice lib-release lib_release )

	IF( ZeroCIceCore_LIBRARY )
		SET( ZeroCIceCore_FOUND TRUE )
	ENDIF( ZeroCIceCore_LIBRARY )


ENDIF( ZeroCIceCore_INCLUDE_DIR )
