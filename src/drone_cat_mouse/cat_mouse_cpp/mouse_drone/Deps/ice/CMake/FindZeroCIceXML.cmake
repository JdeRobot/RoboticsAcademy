# Find the ZeroC ICEXML includes and libraries

#
# ZeroCIceXML_INCLUDE_DIR
# ZeroCIceXML_LIBRARIES
# ZerocCIceCore_FOUND


#
# Copyright (c) 2007, Pau Garcia i Quiles, <pgquiles@elpauer.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

FIND_PATH( ZeroCIceXML_INCLUDE_DIR NAMES IceXML/Parser.h PATHS ENV C++LIB ENV PATH PATH_SUFFIXES include Ice Ice/include )

IF( ZeroCIceXML_INCLUDE_DIR )
	FIND_LIBRARY( ZeroCIceXML_LIBRARY NAMES IceXML PATHS ENV C++LIB ENV PATH PATH_SUFFIXES Ice lib-release lib_release )

	IF( ZeroCIceXML_LIBRARY )
		SET( ZeroCIceXML_FOUND TRUE )
	ENDIF( ZeroCIceXML_LIBRARY )


ENDIF( ZeroCIceXML_INCLUDE_DIR )
