## Setup
- Clone the [Robotics Academy](https://github.com/JdeRobot/RoboticsAcademy) repository on Local Machine

## How to run the exercise
- Pull the latest docker image
    ```bash
    docker pull jderobot/robotics-academy:0.1
    ```

- Start a new docker container of the image

    ```bash
    docker run -it --name=docker_academy -p 8080:8080 -p 7681:7681 -p 2303:2303 -p 1905:1905 jderobot/robotics-academy
    ```

- Update the RoboticsAcademy and CustomRobots repository

    ```bash
    # Updating the Robotics Academy Repo
    cd RoboticsAcademy && git pull origin master

    # Updating the Custom Robots Repo
    cd /opt/jderobot/CustomRobots && git pull origin melodic-devel
    ```

- Export Gazebo environment variables

    ```bash
    cd /RoboticsAcademy/exercises/obstacle_avoidance/web-template/launch
    export GAZEBO_RESOURCE_PATH=$PWD

    cd /opt/jderobot/CustomRobots/f1
    export GAZEBO_MODEL_PATH=$PWD
    ```

- Update the models repository

    ```bash
    cd /gzweb
    npm run deploy --- -m
    ```

- Navigate the web templates based obstacle_avoidance exercise(inside the docker bash)

    ```bash
    cd /RoboticsAcademy/exercises/obstacle_avoidance/web-template/
    ```

- Launch the headless simulation of the exercise(inside the docker bash)

    ```bash
    roslaunch ./launch/obstacle_avoidance_f1_headless.launch
    ```

- The last instruction runs a process that must not be stopped. To carry out the next steps we need to open the container bash in another terminal window.

    ```bash
    docker exec -it docker_academy bash
    ```

- Launch the Gzweb client to enable the Gazebo Simulation to be viewed outside the container. After this instruction, the Gazebo world can be viewed in the native machine's browser from the IP address of the Docker with 8080 as the port.

    ```bash
    cd gzweb
    npm start -p 8080
    ```

- The last instruction will also keep on running. To execute the last step we open a new container bash in another terminal window

    ```bash
    docker exec -it docker_academy bash
    ```

- Determine the IP address with which the container is communicating with the native machine. The list of hosts can be found by the following command. Generally, the IP address is `172.17.0.x`

    ```bash
    cat /etc/hosts
    ```

- Navigate to the exercise folder

    ```bash
    cd RoboticsAcademy/exercises/obstacle_avoidance/web-template
    ```

- Run the academy application

    ```bash
    python host.py <IP address found above>
    ```

- The last instruction will run indefinetly too. On  our local machine navigate to the follow_line exercise which is: `RoboticsAcademy/exercises/obstacle_avoidance/web-template`

- Inside `assets\websocket_address.js` , change the variable `websocket_address` to the IP address through which the container is connected.

- Launch the `index.html` web-page.

## Debug
If while running the exercise you only get a black screen you need to stop the container and start it again (docker container start docker_academy). Run these instructions before starting to follow the **Run the exercise** instructions:

```bash
docker exec -it docker_academy bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
apt-get update && sudo apt-get install -y nvidia-container-toolkit
apt-get install xvfb
Xvfb :1 -screen 0 1600x1200x16 & export Display=:1.0
```