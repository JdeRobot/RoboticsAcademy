# Find the ZeroC ICE executables: 
#
# dumpdb, glacier2router, icebox, iceboxadmin, icecpp, icegridadmin, 
# icegridnode, icegridregistry, icepatch2calc, icepatch2client, 
# icepatch2server, icestormadmin, slice2cpp, slice2cs, slice2docbook, 
# slice2freeze, slice2freezej, slice2html, slice2java, slice2py, 
# slice2rb, slice2vb, transformdb
#
# Sets ZeroCIceExecutables_FOUND to TRUE only if *any* of the executables
# in the ICE_EXECUTABLES were found, therefore you must also check 
# if ZeroCIce_XXXXX_FOUND is true for the executable you want
#
# Defines a ZeroCIce_XXXXX_FOUND and ZeroCIce_XXXXX_BIN variable for each
# executable in the ICE_EXECUTABLES list (the _BIN is the location of the 
# executable)
#
# Defines a ZeroCIce_slice_ICES variable with the location of Plugin.ice, 
# Logger.ice, etc
#
# Copyright (c) 2007, Pau Garcia i Quiles, <pgquiles@elpauer.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

SET( ICE_EXECUTABLES dumpdb glacier2router icebox iceboxadmin icecpp icegridadmin icegridnode icegridregistry icepatch2calc icepatch2client icepatch2server icestormadmin slice2cpp slice2cs slice2docbook slice2freeze slice2freezej slice2html slice2java slice2py slice2rb slice2vb transformdb )

FOREACH( exec ${ICE_EXECUTABLES} )

	FIND_PROGRAM( ZeroCIce_${exec}_BIN NAMES ${exec} PATHS ENV C++LIB ENV PATH PATH_SUFFIXES bin bin_release bin-release )
	FIND_PATH( ZeroCIce_slice_ICES NAMES Ice/Plugin.ice PATHS /usr/share ENV C++LIB ENV PATH PATH_SUFFIXES slice Ice/slice )
#	MESSAGE( "ZeroCIce_slice_ICES = ${ZeroCIce_slice_ICES}" )
	
	IF( ZeroCIce_${exec}_BIN )

		SET( ZeroCIce_${exec}_FOUND TRUE)
		SET( ZeroCIceExecutables_FOUND TRUE )
	ENDIF()

ENDFOREACH( exec )
