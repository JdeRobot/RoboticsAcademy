# Find the ZeroC ICESSL includes and libraries

#
# ZeroCIceSSL_INCLUDE_DIR
# ZeroCIceSSL_LIBRARIES
# ZerocCIceCore_FOUND


#
# Copyright (c) 2007, Pau Garcia i Quiles, <pgquiles@elpauer.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

FIND_PATH( ZeroCIceSSL_INCLUDE_DIR NAMES IceSSL/Plugin.h PATHS ENV C++LIB ENV PATH PATH_SUFFIXES include Ice Ice/include )

IF( ZeroCIceSSL_INCLUDE_DIR )
	FIND_LIBRARY( ZeroCIceSSL_LIBRARY NAMES IceSSL PATHS ENV C++LIB ENV PATH PATH_SUFFIXES Ice lib-release lib_release )

	IF( ZeroCIceSSL_LIBRARY )
		SET( ZeroCIceSSL_FOUND TRUE )
	ENDIF( ZeroCIceSSL_LIBRARY )


ENDIF( ZeroCIceSSL_INCLUDE_DIR )
