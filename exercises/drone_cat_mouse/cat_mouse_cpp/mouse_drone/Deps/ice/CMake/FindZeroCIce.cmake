# Find the ZeroC ICE includes and libraries for every module (Ice, IceStorm, IceUtil, etc)

#
# ZeroCIce_INCLUDE_DIR - Where the includes are. If everything is all right, ZeroCIceXXXX_INCLUDE_DIR is always the same. You usually will use this.
# ZeroCIce_LIBRARIES - List of *all* the libraries. You usually will not use this but only ZeroCIceUtil_LIBRARY or alike
# ZerocCIce_FOUND - True if the core Ice was found
# ZeroCIceCore_FOUND
# ZeroCIceCore_INCLUDE_DIR
# ZeroCIceCore_LIBRARY
# ZeroCIceBox_FOUND
# ZeroCIceBox_INCLUDE_DIR
# ZeroCIceBox_LIBRARY
# ZeroCIceGrid_FOUND
# ZeroCIceGrid_INCLUDE_DIR
# ZeroCIceGrid_LIBRARY
# ZeroCIcePatch2_FOUND
# ZeroCIcePatch2_INCLUDE_DIR
# ZeroCIcePatch2_LIBRARY
# ZeroCIceSSL_FOUND
# ZeroCIceSSL_INCLUDE_DIR
# ZeroCIceSSL_LIBRARY
# ZeroCIceStorm_FOUND
# ZeroCIceStorm_INCLUDE_DIR
# ZeroCIceStorm_LIBRARY
# ZeroCIceUtil_FOUND
# ZeroCIceUtil_INCLUDE_DIR
# ZeroCIceUtil_LIBRARY
# ZeroCIceXML_FOUND
# ZeroCIceXML_INCLUDE_DIR
# ZeroCIceXML_LIBRARY
# ZeroCIceExecutables_FOUND

#
# Copyright (c) 2007, Pau Garcia i Quiles, <pgquiles@elpauer.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

SET( ZeroCIceCore_FIND_QUIETLY TRUE )
SET( ZeroCIceBox_FIND_QUIETLY TRUE )
SET( ZeroCIceGrid_FIND_QUIETLY TRUE )
SET( ZeroCIcePatch2_FIND_QUIETLY TRUE )
SET( ZeroCIceSSL_FIND_QUIETLY TRUE )
SET( ZeroCIceStorm_FIND_QUIETLY TRUE )
SET( ZeroCIceUtil_FIND_QUIETLY TRUE )
SET( ZeroCIceXML_FIND_QUIETLY TRUE )
SET( ZeroCIceExecutables_FIND_QUIETLY TRUE )

FIND_PACKAGE( ZeroCIceCore )
FIND_PACKAGE( ZeroCIceBox )
FIND_PACKAGE( ZeroCIceGrid )
FIND_PACKAGE( ZeroCIcePatch2 )
FIND_PACKAGE( ZeroCIceSSL )
FIND_PACKAGE( ZeroCIceStorm )
FIND_PACKAGE( ZeroCIceUtil )
FIND_PACKAGE( ZeroCIceXML )
FIND_PACKAGE( ZeroCIceExecutables )

SET( ZeroCIce_INCLUDE_DIR ${ZeroCIceCore_INCLUDE_DIR} )
SET( ZeroCIce_LIBRARIES ${ZeroCIceCore_LIBRARY} ${ZeroCIceBox_LIBRARY} ${ZeroCIceGrid_LIBRARY} ${ZeroCIcePatch2_LIBRARY} ${ZeroCIceSSL_LIBRARY} ${ZeroCIceStorm_LIBRARY} ${ZeroCIceUtil_LIBRARY} ${ZeroCIceXML_LIBRARY} )

FOREACH( exec ${ICE_EXECUTABLES} )
	IF(ZeroCIce_${exec}_FOUND)
		LIST(APPEND ZeroCIce_EXECUTABLES ${ZeroCIce_${exec}_BIN} )
	ENDIF(ZeroCIce_${exec}_FOUND)
ENDFOREACH( exec ${ICE_EXECUTABLES} )

SET( ZeroCIce_FOUND ${ZeroCIceCore_FOUND} )

