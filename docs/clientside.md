# CLIENT SIDE

- [RoboticsAcademy frontend](#RoboticsAcademy-frontend)
- [Protocol between manager.py and browser](#Protocol-between-manager.py-and-browser)
- [Protocol between exercise.py and browser](#Protocol-between-exercise.py-and-browser)
- [Protocol between manager.py and browser](#Protocol-between-manager.py-and-browser)
- [Frontend-backend communication](#Frontend-backend-communication)
- [User code processing](#User-code-processing)
- [Flow control](#Flow-control)
- [Types of backend](#Types-of-backend)

<a name="RoboticsAcademy-frontend"></a>
## RoboticsAcademy frontend
RoboticsAcademy frontend is served from an AWS machine using Django, Nginx and MySQL. All the containers are set up using Docker Compose. The exercises are developed on a one single repository (RoboticsAcademy/exercises). The users and the exercises available can be declared and modified through the Django administrator page.
Instructions to declare a new exercise:
1) Access the Django admin page.
2) Click on "Exercises".
3) Click on "add exercise" and fill the fields: exercise id (folder name), name (name to display), state, language and description (description to display).
4) Save and exit.

<a name="Protocol-between-manager.py-and-browser"></a>
## RoboticsAcademy backend
RoboticsAcademy backend is built upon Robotics Academy Docker Image (RADI). This docker image has several exercises available using Gazebo and STDR. In order to request and interact with the exercises, the container has a websocket port (8765) and a communication protocol (Robotics Academy Manager Protocol, or RAMP). Each exercise opens 1, 2 or more websockets to interact specifically with the exercise and receive data.
The RAMP includes these commands:
- “open” in order to start an exercise specified on the field “exercise”
- “stop” to stop the simulation
- “resume” to resume the simulation
- “reset” to reset the simulation
- “evaluate” to request an evaluation of the code sent on the field “code”
- “startgz” to open the viewer GZClient
- “stopgz” to close the viewer GZClient
- “Ping” or "PingDone" to send Ping messages and to communicate that an order has been executed (ater resume, reset or stop commands)

Each exercise is composed of an exercise.html and an exercise.py. The exercise.py is running inside the RADI whereas the exercise.html comes from the browser. Both communicate through websockets: bidirectional communication channels that allow communication between different programming languages.
Each exercise websocket (typically one for the GUI and one for the robot brain) has its own protocol. The first five characters are used to identify the type of the message.

<a name="Protocol-between-exercise.py-and-browser"></a>
## **Protocol between exercise.py and browser**
#### **Code websocket**
The code websocket is used to interact with the brain of the robot, user's source code is sent from the browser to the exercise.py through this websocket. It has a standardized protocol, therefore in the majority of exercises, the code websocket (on port 1905) has these commands available:
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
The GUI websocket (on port 2303) is commonly used to send data from the backend to the frontend since exercise.py has no graphical interface. Gui websocket sends images to the browser to be displayed (e.g: robot camera) and displays them with html widgets. This websocket varies according to the requirements of each exercise, it has more variability. The data is included on a message starting with “#gui” and has different fields based on the exercise. The most common fields are:
- “#image” with the image obtained from the camera of the robot
- “#map” with the position and rotation of the robot

<a name="Protocol-between-manager.py-and-browser"></a>
## **Protocol between manager.py and browser**
#### **Manager websocket**
The manager websocket is in charge of requesting the exercises and handling the control of the simulation (On port 6080, listening to VNC server on 5900).. For example, it starts/stop/pause/resume/kill the Gazebo simulation of the exercise chosen. It also starts the VNC server. The code written by the user is sent first from the browser to the manager.py process through the manager websockets. Then, the manager.py checks the code with Pylint and returns the result to the browser.
It can handle the following commands:
- Open: Kills the previous simulation and starts a new one depending the exercise choosed and if the accelerated simulation is enabled or not.
- Resume: Unpauses gazebo physics
- Stop: Pauses gazebo physics
- Evaluate: Checks the users’ code sent to the manager.py and returns an empty array if there are no errors.
- Evaluate_Style: Checks the users’ code sent to the manager.py and returns an empty array if there are no errors. In this case, it also returns warnings.
- Start: Unpauses gazebo physics
- Reset: Type of reset = default. If the exercise is included in the DRONE_EX array it kills the exercise.py and reset the drone. If the exercise is included in the HARD_RESET_EX it requires to throw everything and built it again.
- Soft reset: Type of reset = soft. In this case, whether the exercise is included in the DRONE_EX array or in the HARD_RESET_EX, the behaviour is the same: it pauses and reset the physics.
- Stopgz: Stops de gazebo client.
- Startgz: Configures the browser screen width and height for the gzclient. It also starts the gazebo client.

<a name="Frontend-backend-communication"></a>
## Frontend-backend communication
The connection between the backend and the frontend consists of these elements:
- websocket manager: request the exercises and control the simulation
- websocket code: interact with the brain of the robot
- websocket GUI: receive data from the robot
- GZClient VNC: interact and visualize the simulation
- Console VNC: debug and print messages

![RoboticsAcademy architecture image](/docs/images/RoboticsAcademy_architecture.png "RoboticsAcademy Architecture")

<a name="User-code-processing"></a>
## User code processing
When a user requests to load the code in the robot, the code follows these steps:
1. The code is sent from the ACE Editor of the browser to the manager.py process of the RADI. The code is checked by Pylint and the errors are returned to the browser. If the browser receives an error, the error is displayed on a modal and the code is not sent to the brain.
2. If there aren't any errors, the code is sent from the browser to the exercise.py through the code websocket. Exercise.py receives the user's source code as raw text and puts it to work.
3. The exercise.py separates the code in two portions: the sequential part (executed once) and the iterative part (executed every brain interval). The code is separated by the first `while True` loop encountered. Within the iterative part the brain measures the time after each
iteration, this is called code management, so as not to saturate the CPU and keep a controlled rhythm of iterations per second to leave the CPU free for other browser tasks. The iterative part is inserted into another template along with extra code that controls the iterations
per second that are carried out, (computational skeleton) so this computational engine is tied to the user code to ensure that the code is executed at a nominal frequency. The user code is also enriched with some execution control elements in order to pause, reset and load a new code into the robot brain.

<a name="Flow-control"></a>
## Flow Control
In order to control the number of messages sent by the users, so the RADI is not overflown with them, both the manager websocket and the code websocket have response messages for certain orders which are sent after the operetions are completed.
- The manager websocket responds with "PingDone" after the operations "start", "stop" or "reset" are completed.
- The code websocket responds with "#exec" after the sent code has been loaded in the brain of the robot.

After any of the previous commands is requested by the user, the respective button is blocked until the completed response returns.

<a name="Types-of-backend"></a>
## Types of backend
There are two types of backend: local and remote
- Local backend: is the default type, available for all users. The user executes the RADI on his own computer.
- Remote backend: available only for specific users. The RADI is run on an external computer. The webserver allows the communication between the external computer and the user broadcasting the messages between them.
