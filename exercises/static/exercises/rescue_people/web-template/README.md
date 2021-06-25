## How to launch the exercise?

- Make sure to have pulled (or built) the last version of the docker image.
```bash
docker pull jderobot/robotics-academy:2.4.3
```

- Open a terminal and run:
```bash
docker --rm run -it --device /dev/dri -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:2.4.3 ./start.sh
```

- On the local machine navigate to 127.0.0.1:8000/ in the browser and choose the desired exercise.

- Click the connect button and wait for some time until an alert appears with the message `Connection Established` and button displays connected.

- The exercise can be used after the alert.