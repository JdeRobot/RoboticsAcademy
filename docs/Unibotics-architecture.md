### [Back to main README.][]

[Back to main README.]: ../README.md

# Unibotics architecture
## Unibotics frontend
Unibotics frontend is served from an AWS machine using Django, Nginx and MySQL. All the containers are set up using Docker Compose. The exercises are developed on a separate repository (unibotics-exercises). The users and the exercises available can be declared and modified through the Django administrator page.
Instructions to declare a new exercise:
1) Access the Django admin page.
2) Click on "Exercises".
3) Click on "add exercise" and fill the fields: exercise id (folder name), name (name to display), state, language and description (description to display).
4) Save and exit.

## Unibotics backend
Unibotics backend is built upon Robotics Academy Docker Image (RADI). This docker image has several exercises available using Gazebo and STDR. In order to request and interact with the exercises, the container has a websocket port (8765) and a communication protocol (Robotics Academy Manager Protocol, or RAMP). Each exercise opens 1, 2 or more websockets to interact specifically with the exercise and receive data.
The RAMP includes these commands:
- “open” in order to start an exercise specified on the field “exercise”
- “stop” to stop the simulation
- “resume” to resume the simulation
- “reset” to reset the simulation
- “evaluate” to request an evaluation of the code sent on the field “code”
- “startgz” to open the viewer GZClient
- “stopgz” to close the viewer GZClient
- “Ping” or "PingDone" to send Ping messages and to communicate that an order has been executed (ater resume, reset or stop commands)

Each exercise websocket (typically one for the GUI and one for the robot brain) has its own protocol. The first five characters are used to identify the type of the message.

#### **Code websocket**
The code websocket is used to interact with the brain of the robot. In the majority of the exercises, the code websocket (on port 1905) has these commands available:
- “#freq” to set the code and gui frequency specified on the fields “brain” and “gui”
- “#code” to update the code of the brain to the new code sent after the tag
- “#play” to start the brain execution
- “#stop” to stop the brain execution
- “#rest” to reset and stop the brain execution
- “#ping” to communicate Ping messages

The server responds with these messages:
- “#exec” response when the last code received with the order “#code” has been loaded in the brain of the robot
- “#freq” message (sent on every iteration of the brain) including three fields: “brain” with the brain frequency; “gui” with the GUI frequency and “rtf” with the Real Time Factor Value.
- “#ping” response to ping messages

#### **GUI Websocket**
The GUI websocket (on port 2303) is commonly used to send data from the backend to the frontend. The data is included on a message starting with “#gui” and has different fields based on the exercise. The most common fields are:
- “#image” with the image obtained from the camera of the robot
- “#map” with the position and rotation of the robot

## Frontend-backend communication
The connection between the backend and the frontend consists of these elements:
- websocket manager: request the exercises and control the simulation
- websocket code: interact with the brain of the robot
- websocket GUI: receive data from the robot
- GZClient VNC: interact and visualize the simulation
- Console VNC: debug and print messages

![unibotics architecture image](/docs/images/unibotics_architecture.png "Unibotics Architecture")

## User code processing
When a user requests to load the code in the robot, the code follows these steps:
1. The code is sent from the ACE Editor of the browser to the manager.py process of the RADI. The code is checked by Pylint and the errors are returned to the browser. If the browser receives an error, the error is displayed on a modal and the code is not sent to the brain.
2. If there aren't any errors, the code is sent from the browser to the exercise.py through the code websocket.
3. The exercise.py separates the code in two portions: the sequential part (executed once) and the iterative part (executed every brain interval). The code is separated by the first `while True` loop encountered. The user code is also enriched with some execution control elements in order to pause, reset and load a new code into the robot brain.

## Flow Control
In order to control the number of messages sent by the users, so the RADI is not overflown with them, both the manager websocket and the code websocket have response messages for certain orders which are sent after the operetions are completed.
- The manager websocket responds with "PingDone" after the operations "start", "stop" or "reset" are completed.
- The code websocket responds with "#exec" after the sent code has been loaded in the brain of the robot.

After any of the previous commands is requested by the user, the respective button is blocked until the completed response returns.


## Types of backend
There are two types of backend: local and remote
- Local backend: is the default type, available for all users. The user executes the RADI on his own computer.
- Remote backend: available only for specific users. The RADI is run on an external computer. The webserver allows the communication between the external computer and the user broadcasting the messages between them.

## How is the code split between repositories
Unibotics depends on 5 repositories. Those repositories are: [unibotics-webserver](https://github.com/JdeRobot/unibotics-webserver), [unibotics-exercises](https://github.com/JdeRobot/unibotics-exercises), [RoboticsAcademy](https://github.com/JdeRobot/RoboticsAcademy), [CustomRobots](https://github.com/JdeRobot/CustomRobots) and [drones](https://github.com/JdeRobot/drones), all owned and maintained by [JdeRobot](https://github.com/JdeRobot). The purpose of all of them are:
- Unibotics Webserver: contains all the files regarding the web server that hosts the Unibotics page (the docker-compose files, the database files, the Django server...)
- Unibotics Exercises: contains the specific pages of all the exercises available on Unibotics. Those files are the frontend files, which communicate with the RADI and offers a friendly interface to the users.
- Robotics Academy: contains the backend of the platform (the RADI). It contains a Dockerfile and all of the scripts that determine how the exercises work, the simulation and the scripts that implements the communication protocols explained above. The RADI is a containerized Gazebo.
- CustomRobots: contains different types of robots (vehicles, service robots...) and world files for Gazebo. It is included in the RADI.
- Drones: contains differnt types of drones and world files for Gazebo. It is included in the RADI.
