#!/bin/bash

# usage: ./xserver.sh [display]
# example: ./xserver.sh :0 

cd /
/usr/bin/Xorg -noreset +extension GLX +extension RANDR +extension RENDER -logfile ./xdummy.log -config /xorg.conf $1