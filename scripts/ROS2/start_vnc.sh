#!/bin/bash

usage() {
    echo 'usage: /start_vnc.sh <display> <internal_port> <external_port> 
        example: >> /start_vnc.sh 0 5900 6080'
}

if [ $# -ne 3 ] ; then
    usage
    exit 1
fi

display=$1
internal_port=$2
external_port=$3

cd /
## xserver:
/usr/bin/Xorg -noreset -quiet +extension GLX +extension RANDR +extension RENDER -logfile ./xdummy.log -config ./xorg.conf :$display &
sleep 1
## lanzar servidor vnc
x11vnc -display :$display -quiet -nopw -forever -xkb -bg -rfbport $internal_port &
sleep 1
## lanzar cliente noVNC
/noVNC/utils/novnc_proxy --listen $external_port --vnc localhost:$internal_port &

export DISPLAY=:$display