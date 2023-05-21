#!/bin/bash

# usage: ./vnc_novnc.sh [display] [in_port] [out_port]
# example: ./vnc_novnc.sh :0 5900 6080
cd /
x11vnc -display $1 -nopw -forever -bg -xkb -rfbport $2
/noVNC/utils/novnc_proxy --listen $3 --vnc localhost:$2