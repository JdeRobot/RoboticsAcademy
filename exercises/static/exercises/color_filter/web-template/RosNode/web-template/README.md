# Color Filter Exercise using WebTemplates

## How to launch the exercise?

- There are two ways to run the exercise using web-template.Either one way is fine.

      - Run the exercise with docker container
      - Run it without container

## Run with docker container

- Jderobot already has created docker image available from [docker hub here](https://hub.docker.com/r/jderobot/robotics-academy/).
- Follow the instructions line by line to run the exercise.

## Run without docker container

- All the generic and specific infrastructures including required libraries need to be installed already on your machine as stated on the [academy webpage here](http://jderobot.github.io/RoboticsAcademy/installation/).

- Requirements to create the RosNode
    - Create a New ROS Package
        ~~~
            cd ~/catkin_ws/src 
            catkin_create_pkg cv_basics image_transport cv_bridge sensor_msgs rospy roscpp std_msgs
        ~~~  
    - Create the Image Publisher Node 
        ~~~
            cd ~/catkin_ws/src
            roscd cv_basics
            mkdir scripts
            cp <exercice>/launch/webcam_pub.py  ~/catkin_ws/src/cv_basics/scripts
            mkdir launch
            cp <exercice>/launch/cv_basics_py.launch  ~/catkin_ws/src/cv_basics/scripts/
        ~~~  
    - Launch the node
        ~~~
            roslaunch cv_basics cv_basics_py.launch
        ~~~ 

- Determine your machine dns server IP address which is generally in the form of **127.0.0.xx for Linux machine** by running this command

```bash
cat /etc/resolv.conf
```

- Inside `assets/websocket_address.js` file, change the **variable websocket_address** to the IP address found with the above command

- Start the host application along with the same IP address which is used for connection.

```bash
python exercise.py 127.0.0.xx
```

- Open the browser template from `exercise.html`

- The page should says **[open]Connection established!**.Means it is working as expected.

**__NOTE:__**  If you get **socket.error: [Errno 99] Cannot assign requested address** error,you need to check and pass the correct IP address.
