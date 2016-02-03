#  Copyright (C) 2015-2016 JDE Developers Team
#  Authors:
#    Victor Arribas <v.arribas.urjc@gmail.com>

## Headers trick for IDEs
# Assumes that all heades are under "include/" directory
# Requires defined PROJECT_NAME

cmake_minimum_required(VERSION 2.8)

file(GLOB_RECURSE ${PROJECT_NAME}_headers "include/**")
if (${PROJECT_NAME}_headers)
    add_library(${PROJECT_NAME}_headers EXCLUDE_FROM_ALL ${${PROJECT_NAME}_headers})
    set_target_properties(${PROJECT_NAME}_headers PROPERTIES LINKER_LANGUAGE CXX)
endif()

