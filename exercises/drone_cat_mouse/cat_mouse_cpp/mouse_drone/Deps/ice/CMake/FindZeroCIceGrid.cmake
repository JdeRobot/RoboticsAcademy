# Find the ZeroC ICEGrid includes and libraries

#
# ZeroCIceGrid_INCLUDE_DIR
# ZeroCIceGrid_LIBRARIES
# ZerocCIceCore_FOUND


#
# Copyright (c) 2007, Pau Garcia i Quiles, <pgquiles@elpauer.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

FIND_PATH( ZeroCIceGrid_INCLUDE_DIR NAMES IceGrid/UserAccountMapper.h PATHS ENV C++LIB ENV PATH PATH_SUFFIXES include Ice Ice/include )

IF( ZeroCIceGrid_INCLUDE_DIR )
	FIND_LIBRARY( ZeroCIceGrid_LIBRARY NAMES IceGrid PATHS ENV C++LIB ENV PATH PATH_SUFFIXES Ice lib-release lib_release )

	IF( ZeroCIceGrid_LIBRARY )
		SET( ZeroCIceGrid_FOUND TRUE )
	ENDIF( ZeroCIceGrid_LIBRARY )


ENDIF( ZeroCIceGrid_INCLUDE_DIR )
