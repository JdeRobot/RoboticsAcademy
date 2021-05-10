# Follow Road Exercise using WebTemplates

There are two ways to run the exercise using web-template. Either one way is fine.

- Run the exercise with docker container
- Run it without container

## Run with docker container

- Jderobot already has created docker image available from [docker hub here](https://hub.docker.com/r/jderobot/robotics-academy/).
- Follow the instructions line by line to run the exercise.

## Run without docker container

- All the generic and specific infrastructures including required libraries need to be installed already on your machine as stated on the [academy webpage here](http://jderobot.github.io/RoboticsAcademy/installation/).

## How to launch the exercise?

- Make sure to have pulled (or built) the last version of the docker image.
```bash
docker pull jderobot/robotics-academy
```

### User launching

- Open a terminal and run:
```bash
docker run -it -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy ./start.sh
```

- On the local machine navigate to 127.0.0.1:8000/ in the browser and choose the desired exercise.

- Click the connect button and wait for some time until an alert appears with the message `Connection Established` and button displays connected.

- The exercise can be used after the alert.

### Developer launching

- Open a terminal and run:
```bash
docker exec -it docker_academy bash

python3 RoboticsAcademy/manage.py runserver 0.0.0.0:8000
```

- Open a terminal and run:
```bash
docker exec -it docker_academy bash

Xvfb :0 -screen 0 1600x1200x16  &
Xvfb :1 -screen 0 1600x1200x16  &

roslaunch /RoboticsAcademy/exercises/static/exercises/follow_road/web-template/launch/follow_road.launch
```

- Open a terminal and run:
```bash
docker exec -it docker_academy bash

x11vnc -display :0 -nopw -forever -xkb -bg -rfbport 5900
/noVNC/utils/launch.sh --listen 6080 --vnc localhost:5900
```

- Open a terminal and run:
```bash
docker exec -it docker_academy bash

x11vnc -display :1 -nopw -forever -xkb -bg -rfbport 5901
/noVNC/utils/launch.sh --listen 1108 --vnc localhost:5901
```

- Open a terminal and run:
```bash
docker exec -it docker_academy bash

gzclient --verbose
```

- Open a terminal and run:
```bash
docker exec -it docker_academy bash

python /RoboticsAcademy/exercises/static/exercises/follow_road/web-template/exercise.py 0.0.0.0
```

- On the local machine navigate to 127.0.0.1:8000/ in the browser and choose the desired exercise.

- Click the connect button and wait for some time until an alert appears with the message `Connection Established` and button displays connected.

- The exercise can be used after the alert.