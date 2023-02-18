#!/bin/bash

############################################################
# Help                                                     #
############################################################
Help()
{
   # Display Help
   echo "Syntax: build.sh [-h] [-f] <tag>"
   echo "options:"
   echo "-h   Print this Help."
   echo "-f   Force creation of base image. If ommited, the base image is created only if "
   echo "     it doestn exist."
   echo
}

force_build=false
while getopts ":hf" option; do
   case $option in
      h) # display Help
         echo "Generates RoboticsAcademy RADI image"
         echo
         Help
         exit 0
         ;;
      f)
        force_build=true
        ;;
      \?)
        echo "Invalid Option: -$OPTARG" 1>&2
        Help
        exit 1
        ;;
   esac
done
shift $(($OPTIND - 1))

image_tag="$1"
if [ -z "$image_tag" ]; then
  echo "You have to specify a tag for the image"
  echo
  Help
  exit 2
fi

if $force_build || [[ "$(docker images -q jderobot/robotics-applications:base 2> /dev/null)" == "" ]]; then
  echo "BUILDING Jderobot BASE IMAGE ====================="
  echo
  docker build -f Dockerfile.base -t jderobot/robotics-applications:base .
fi

echo "BUILDING RoboticsAcademy RADI IMAGE ====================="
echo
docker build --no-cache=true -t jderobot/robotics-academy:$image_tag .
