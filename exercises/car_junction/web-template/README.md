# Car Junction Exercise using WebTemplates

## How to launch the exercise?

For users of Robotics Academy, follow the instructions given on this [link](http://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/car_junction/)

Note: For Windows users running the exercise through the web template, the websocket_address would be the IP address of the **vEthernet adapter(WSL)**. This can be found through the following command at the command prompt:
```
ipconfig/all
```

## Dependencies

[Opel models](https://github.com/JdeRobot/CustomRobots/tree/melodic-devel/car_junction/opel_ROS) from CustomRobots repository

### Launch Files

`launch` folder contains the various launch files and world files, for running the Gazebo simulation of the exercise

### Asset Files

`assets` folder contains the Javascript code for the webpage part of the web templates

### Python Files

`exercise.py` is the main Python file to run the exercise


### Launch the gazebo simulation

```bash
roslaunch ./launch/car_junction.launch
```

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
