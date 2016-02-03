#  Copyright (C) 2016 JDE Developers Team
#  Authors:
#    Victor Arribas <v.arribas.urjc@gmail.com>


## A fast tool for inject dependencies found by find_package
# Relies on ${PACKAGE}_FOUND to allow external custom call to find_package()
# Compatible with declare_package
#
macro(use_package PACKAGE)
    if (NOT ${PACKAGE}_FOUND)
        find_package(${PACKAGE} QUIET)
    endif()

    if (${PACKAGE}_FOUND)
        include_directories(
            ${${PACKAGE}_INCLUDE_DIRS}
            ${${PACKAGE}_INCLUDE_DIR}
        )

        link_directories(
            ${${PACKAGE}_LIBRARY_DIRS}
            ${${PACKAGE}_LIBRARY_DIR}
        )

        list(APPEND LIBRARIES
            ${${PACKAGE}_LIBRARIES}
        )
    else()
        if (NOTFOUND_IS_ERROR)
            message(SEND_ERROR "${PROJECT_NAME}: dependency '${PACKAGE}' not found")
        endif()
    endif()
endmacro()

