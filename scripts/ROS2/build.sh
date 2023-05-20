#!/bin/sh

image_tag="$1"
if [ -z "$image_tag" ]; then
  echo "You have to specify a tag for the image"
  echo
  Help
  exit 2
fi

echo "BUILDING RoboticsAcademy RADI IMAGE ====================="
echo
docker build -f Dockerfile.academy --no-cache=true -t jderobot/robotics-academy:$image_tag .
