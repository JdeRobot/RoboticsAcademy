#!/bin/sh
#
# Copyright (c) 2016
# Author: Victor Arribas <v.arribas.urjc@gmail.com>
# License: GPLv3 <http://www.gnu.org/licenses/gpl-3.0.html>

world=./cityLarge.world

cd gazebo
gzserver --verbose --minimal_comms $world &
sleep 10 # up to 20 for circuit.world
cd ..
[ "$1" = "GUI" ] && gzclient &

python main.py --mapConfig=taxiMap.conf --Ice.Config=teleTaxi.cfg

killall gzserver
[ "$1" = "GUI" ] && killall gzclient
