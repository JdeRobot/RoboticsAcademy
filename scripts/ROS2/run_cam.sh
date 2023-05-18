#!/bin/bash

usage() {
    echo "usage: run.sh : tag must be specified
            -t | --tag : docker image tag (required)
            -v | --volume : route to a shared directory (optional)
            -n | --name : name of container (optional)
            -d | --debug : open a terminal inside container (optional)
            --dev : use device inside container (optional) [specify device name]
            
            example of usage:
                ./run.sh -t 4.3.0 -v /home/user/dir -n my_container -d --dev card0
            "
}


volume=false
name=""
tag=""
unset route
unset debug
unset device
unset device_name

while [[ "$1" =~ ^- && ! "$1" == "--" ]]; do case $1 in
    -v | --volume )
        shift;
        route=$1
        volume=true
        ;;
    -n | --name )
        shift; 
        name=$1
        ;;
    -d | --debug )
        debug=bash
        ;;
    --dev )
        shift;
        device="--device /dev/dri"
        device_name=$1
        ;;
    -t | --tag )
        shift;
        tag=$1
        ;;
esac; shift; done
if [[ "$1" == '--' ]]; then shift; fi

if [ -z $tag ]; then
    usage
    exit 1
fi

if [ -z $name ]; then
    name=dockerRam_container
    echo name not specified, container name will be: dockerRam_container
fi

if $volume && [ ! -z $route ] && [ -d $route ] ; then
    docker run --name $name $device -e DRI_NAME=$device_name -v $route:/home/shared_dir --rm -it -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 6081:6081 -p 1108:1108 -p 6082:6082 -p 7163:7163 --device /dev/video0:/dev/video0 jderobot/robotics-academy:$tag $debug
else
    docker run --name $name $device -e DRI_NAME=$device_name --rm -it -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 6081:6081 -p 1108:1108 -p 6082:6082 -p 7163:7163 --device /dev/video0:/dev/video0 jderobot/robotics-academy:$tag $debug
fi