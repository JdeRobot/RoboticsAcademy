#  Copyright (C) 2015 JdeRobot Developers Team
#  Authors:
#    Victor Arribas <v.arribas.urjc@gmail.com>

## The hard way (no help)

set(libname easyiceconfig)

find_path(${libname}_INCLUDE_DIR NAMES easyiceconfig/EasyIce.h
)
find_library(${libname}_LIBRARY NAMES ${libname}
    PATH_SUFFIXES ${libname} jderobot
)

if (${libname}_INCLUDE_DIR AND ${libname}_LIBRARY)
    get_filename_component(${libname}_LIBRARY_DIR ${${libname}_LIBRARY} PATH)
    message(STATUS "${libname} FOUND")
    set(${libname}_FOUND 1)
    set(${libname}_INCLUDE_DIRS ${${libname}_INCLUDE_DIR})
    set(${libname}_LIBRARY_DIRS ${${libname}_LIBRARY_DIR})
    set(${libname}_LIBRARIES    ${${libname}_LIBRARY})
else()
    message(WARNING "${libname} NOT FOUND")
endif()


if(${libname}_FOUND AND INJECT_JDE_LIBS)
    include_directories(${${libname}_INCLUDE_DIRS})
    link_directories(${${libname}_LIBRARY_DIRS})
endif()

unset(libname)

