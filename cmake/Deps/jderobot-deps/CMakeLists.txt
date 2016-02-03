#  Copyright (C) 2015 JdeRobot Developers Team
#  Authors:
#    Victor Arribas <v.arribas.urjc@gmail.com>

find_file(LoadJderobotDependencies NAMES LoadJderobotDependencies.cmake
	PATHS /usr/share/jderobot /usr/local/share/jderobot
	PATH_SUFFIXES Deps
)
if (LoadJderobotDependencies-NOTFOUND)
	message(WARNING "LoadJderobotDependencies not found. using default path")
	set(LoadJderobotDependencies /usr/local/share/jderobot/Deps/LoadJderobotDependencies.cmake)
endif()

include(${LoadJderobotDependencies})
