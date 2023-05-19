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
# TurvoVNC (xserver + vncserver)
export VGL_DISPLAY=/dev/dri/card0 && export TVNC_WM=startlxde && /opt/TurboVNC/bin/vncserver :$display -geometry '1920x1080' -vgl -noreset -SecurityTypes None -rfbport $internal_port & 
sleep 1
# lanzar cliente noVNC
/noVNC/utils/novnc_proxy --listen $external_port --vnc localhost:$internal_port &