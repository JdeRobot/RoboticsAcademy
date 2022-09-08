#!/bin/bash

result=${PWD##*/}          # to assign to a variable
result=${result:-/}        # to correct for the case where PWD=/
runMethod="dev"
bWrt='starting react development'

if [ "$result" != "RoboticsAcademy" ]
then
  echo "Check root folder path"
  exit 1;
fi

case "$1" in
"build" ) bWrt='building react frontend'; runMethod="build";;
esac

echo "Step 1: Launching Robotics Academy Docker Image [RADI]"

docker run --rm -i -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy --no-server &

P1=$!

echo "Step 2: $bwrt"
cd react_frontend || exit
yarn run "$runMethod" &

P2=$!
echo "Step3 : starting Django Server "
cd .. || exit
source env/bin/activate
python3 manage.py runserver && fg

P3=$!

echo " Start learning !!"






