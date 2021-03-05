# Drone Cat Mouse Exercise using WebTemplates

There are two ways to run the exercise using web-template. Either one way is fine.

- Run the exercise with docker container
- Run it without container

## Run with docker container

- Jderobot already has created docker image available from [docker hub here](https://hub.docker.com/r/jderobot/robotics-academy/).
- Follow the instructions line by line to run the exercise.

## Run without docker container

- All the generic and specific infrastructures including required libraries need to be installed already on your machine as stated on the [academy webpage here](http://jderobot.github.io/RoboticsAcademy/installation/).

## How to launch the exercise?

### First execution only
- Build or pull the docker image.
- Open a terminal and run:
```bash
docker run -it --name=docker_academy -p 8080:8080 -p 7681:7681 -p 2303:2303 -p 1905:1905 jderobot/robotics-academy:drones-beta
npm run deploy --- -m local
exit
```

- Add jderobot-drones by source (provisional):
```bash
docker exec -it docker_academy bash

apt-get install python-catkin-tools
 
mkdir -p /catkin_ws/src
cd /catkin_ws
catkin init
echo 'export ROS_WORKSPACE=/catkin_ws' >> ~/.bashrc # points roscd dir
source ~/.bashrc

git clone https://github.com/JdeRobot/drones.git
roscd && cd src
ln -s /drones/drone_wrapper .

roscd
rosdep init  # needs to be called ONLY once after installation. sudo might be required
rosdep update && rosdep install --from-paths . --ignore-src --rosdistro melodic -y  #sudo might be required

roscd && catkin build

exit
```


### All the executions
- Open a terminal and run:
```bash
docker exec -it docker_academy bash

Xvfb :1 -screen 0 1600x1200x16  &
export DISPLAY=:1.0
cd /RoboticsAcademy/exercises/drone_cat_mouse/web-template
roslaunch launch/drone_cat_mouse.launch
```

- Open a terminal and run:
```bash
docker exec -it docker_academy bash

cd /gzweb && npm start -p 8080
```

- Open a terminal and run:
```bash
docker exec -it docker_academy bash

source /catkin_ws/devel/setup.bash
python /RoboticsAcademy/exercises/drone_cat_mouse/web_template/exercise.py 0.0.0.0
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
