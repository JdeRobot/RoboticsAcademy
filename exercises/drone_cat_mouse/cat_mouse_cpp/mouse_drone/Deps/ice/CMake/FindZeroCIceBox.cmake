# Find the ZeroC ICEBox includes and libraries

#
# ZeroCIceBox_INCLUDE_DIR
# ZeroCIceBox_LIBRARIES
# ZeroCIceBox_FOUND


#
# Copyright (c) 2007, Pau Garcia i Quiles, <pgquiles@elpauer.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

FIND_PATH( ZeroCIceBox_INCLUDE_DIR NAMES IceBox/IceBox.h PATHS ENV C++LIB ENV PATH PATH_SUFFIXES include Ice Ice/include )

IF( ZeroCIceBox_INCLUDE_DIR )
	FIND_LIBRARY( ZeroCIceBox_LIBRARY NAMES IceBox PATHS ENV C++LIB ENV PATH PATH_SUFFIXES Ice lib-release lib_release )

	IF( ZeroCIceBox_LIBRARY )
		SET( ZeroCIceBox_FOUND TRUE )
	ENDIF( ZeroCIceBox_LIBRARY )


ENDIF( ZeroCIceBox_INCLUDE_DIR )
