# JdeRobot Academy: Instructions to start the docker image

Build your own docker image to start simulations

## Installation

First you need to build the image. Then, you need to run a container.

```sh
git clone -b foxy https://github.com/JdeRobot/RoboticsAcademy.git
cd RoboticsAcademy/scripts
chmod +x build.sh
./build.sh
# Building might take 40-50 minutes
docker run -it \
      --rm \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      --name foxy_radi_container \
      -v $PWD/../:/RoboticsAcademy \
      -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 \
      foxy-radi ./start.sh
```

## Direct Method: Via DockerHub
```sh
docker run -it \
      --rm \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      --name foxy_radi_container \
      -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 \
      jderobot/robotics-academy:4.0.0 ./start.sh
```
