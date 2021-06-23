#!/usr/bin/env bash

# This file is intended to be run from the root
# of the Docker container


# radi-entrypoint gets populated solely through
# the Dockerfile.
# Its primary purpose is to source the ROS setup files.
source radi-entrypoint.sh

# Start the Django server. Load the list of exercises
# from db.sqlite3
python3 RoboticsAcademy/manage.py runserver 0.0.0.0:8000 &

# Start RADI
python3.8 manager.py