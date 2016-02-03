#  Copyright (C) 2015 JDE Developers Team
#  Authors:
#    Victor Arribas <v.arribas.urjc@gmail.com>

## Export library variables in find_package format
# requires headers to be placed inside include/ tree
macro(declare_package PROJECT_NAME PROJECT_LIBS)
    ## declare at cmake's cache
    # enforced to override any previous cache assignment (=prefer build over system)
    # it isn't enough if was "already manually" set (like using -D)
    set(${PROJECT_NAME}_FOUND 1 CACHE BOOL "Find(${PROJECT_NAME})" FORCE)
    set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE PATH "Find(${PROJECT_NAME})" FORCE)
    set(${PROJECT_NAME}_LIBRARY_DIRS "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "Find(${PROJECT_NAME})" FORCE)
    set(${PROJECT_NAME}_LIBRARIES "${PROJECT_LIBS}" CACHE STRINGS "Find(${PROJECT_NAME})" FORCE)

    ## declare at parent scope
    # normal step after add_subdirectory()
    set(${PROJECT_NAME}_FOUND 1 PARENT_SCOPE)
    set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include" PARENT_SCOPE)
    set(${PROJECT_NAME}_LIBRARY_DIRS "${CMAKE_CURRENT_BINARY_DIR}" PARENT_SCOPE)
    set(${PROJECT_NAME}_LIBRARIES "${PROJECT_LIBS}" PARENT_SCOPE)

    ## declare at current scope
    set(${PROJECT_NAME}_FOUND 1)
    set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")
    set(${PROJECT_NAME}_LIBRARY_DIRS "${CMAKE_CURRENT_BINARY_DIR}")
    set(${PROJECT_NAME}_LIBRARIES "${PROJECT_LIBS}")
endmacro()


macro(declare_variable package_VARIABLE value type)
    set(${package_VARIABLE} ${value} CACHE ${type} "Find(${PROJECT_NAME})" FORCE)
    set(${package_VARIABLE} ${value} PARENT_SCOPE)
    set(${package_VARIABLE} ${value})
endmacro()

