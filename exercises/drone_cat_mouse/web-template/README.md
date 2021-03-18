# Follow Line Exercise using WebTemplates

## How to launch the exercise?

- There are two ways to run the exercise using web-template.Either one way is fine.

      - Run the exercise with docker container
      - Run it without container

## Run with docker container

- Jderobot already has created docker image available from [docker hub here](https://hub.docker.com/r/jderobot/robotics-academy/).
- Follow the instructions line by line to run the exercise.

## Run without docker container

- All the generic and specific infrastructures including required libraries need to be installed already on your machine as stated on the [academy webpage here](http://jderobot.github.io/RoboticsAcademy/installation/).

### Clone CustomRobots repo and load models from it

```bash
cd /opt/jderobot
git clone https://github.com/JdeRobot/CustomRobots.git
```

- Export Gazebo environment variables

```bash
cd CustomRobots/f1
export GAZEBO_MODEL_PATH=$PWD
```

```bash
cd && cd /RoboticsAcademy/exercises/follow_line/web-template/launch
export GAZEBO_RESOURCE_PATH=$PWD
cd ..
```

### Launch the gazebo simulation

```bash
roslaunch ./launch/simple_line_follower_ros.launch
```

- Determine your machine dns server IP address which is generally in the form of **127.0.0.xx for Linux machine** by running this command

```bash
cat /etc/resolv.conf
```

- Inside `assets/websocket_address.js` file, change the **variable websocket_address** to the IP address found with the above command 

- Start the host application along with the same IP address which is used for connection.

```bash
python host.py 127.0.0.xx
```

- Open the browser template from `index.html`

- The page should says **[open]Connection established!**.Means it is working as expected.

**__NOTE:__**  If you get **socket.error: [Errno 99] Cannot assign requested address** error,you need to check and pass the correct IP address.


