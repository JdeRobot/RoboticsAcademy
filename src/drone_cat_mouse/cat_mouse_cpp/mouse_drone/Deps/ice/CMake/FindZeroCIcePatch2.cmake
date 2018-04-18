# Find the ZeroC ICEPatch2 includes and libraries

#
# ZeroCIcePatch2_INCLUDE_DIR
# ZeroCIcePatch2_LIBRARIES
# ZerocCIceCore_FOUND


#
# Copyright (c) 2007, Pau Garcia i Quiles, <pgquiles@elpauer.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

FIND_PATH( ZeroCIcePatch2_INCLUDE_DIR NAMES IcePatch2/ClientUtil.h PATHS ENV C++LIB ENV PATH PATH_SUFFIXES include Ice Ice/include )

IF( ZeroCIcePatch2_INCLUDE_DIR )
	FIND_LIBRARY( ZeroCIcePatch2_LIBRARY NAMES IcePatch2 PATHS ENV C++LIB ENV PATH PATH_SUFFIXES Ice lib-release lib_release )

	IF( ZeroCIcePatch2_LIBRARY )
		SET( ZeroCIcePatch2_FOUND TRUE )
	ENDIF( ZeroCIcePatch2_LIBRARY )


ENDIF( ZeroCIcePatch2_INCLUDE_DIR )
