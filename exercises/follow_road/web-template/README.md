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
docker run -it -p 8080:8080 -p 7681:7681 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 jderobot/robotics-academy python3.8 manager.py
```

- Open `exercise.html` on you web browser.

- The page should says **[open]Connection established!**. Means it is working as expected.

### Developer launching
- Open a terminal and run:
```bash
docker exec -it docker_academy bash

Xvfb :1 -screen 0 1600x1200x16  &
export DISPLAY=:1.0
cd /RoboticsAcademy/exercises/follow_road/web-template
roslaunch launch/follow_road.launch
```

- Open a terminal and run:
```bash
docker exec -it docker_academy bash

cd /gzweb && npm start -p 8080
```

- Open a terminal and run:
```bash
docker exec -it docker_academy bash

python /RoboticsAcademy/exercises/follow_road/web_template/exercise.py 0.0.0.0
```

- Determine your machine dns server IP address which is generally in the form of **127.0.0.xx for Linux machine** by running this command:
```bash
docker exec -it docker_academy bash

cat /etc/resolv.conf
```

- Inside `assets/websocket_address.js` file, change the **variable websocket_address** to the IP address found with the above command.

- Open `exercise.html` on you web browser.

- The page should says **[open]Connection established!**. Means it is working as expected.

**__NOTE:__**  If you get **socket.error: [Errno 99] Cannot assign requested address** error,you need to check and pass the correct IP address.
