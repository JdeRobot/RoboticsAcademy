### [Back to main README.][]

[Back to main README.]: ../README.md

# CLIENT SIDE

- [Robotics Academy frontend](#Robotics-Academy-frontend)
- [Robotics Academy backend](#Robotics-Academy-backend)
- [Protocol between exercise.py and browser](#Protocol-between-exercise.py-and-browser)
- [Protocol between manager.py and browser](#Protocol-between-manager.py-and-browser)
- [Frontend-backend communication](#Frontend-backend-communication)
- [User code processing](#User-code-processing)
- [Threads used in the exercises](#Threads-used-in-the-exercises)
- [Flow control](#Flow-control)
- [Manager.py extended](#Manager.py extended)

<a name="Robotics-Academy-frontend"></a>
## Robotics Academy frontend
Robotics frontend is served from a Django webserver running inside the RADI. Each exercise page communicates with different elements of the RADI in order to interact with the simulation.

<a name="Robotics-Academy-backend"></a>
## Robotics Academy backend
The **Robotics Academy Docker Image** (RADI) has several exercises available using **Gazebo** and **STDR**. In order to request and interact with the exercises, the container has a websocket **port (8765)** and a communication protocol (Robotics Academy Manager Protocol, or RAMP). Each exercise opens 1, 2 or more websockets to interact specifically with the exercise and receive data. The RAMP includes these commands:

    “open” in order to start an exercise specified on the field “exercise”
    “stop” to stop the simulation
    “resume” to resume the simulation
    “reset” to reset the simulation
    “evaluate” to request an evaluation of the code sent on the field “code”
    “startgz” to open the viewer GZClient
    “stopgz” to close the viewer GZClient
    “Ping” or "PingDone" to send Ping messages and to communicate that an order has been executed (ater resume, reset or stop commands)

Each exercise is composed of an **exercise.html** and an **exercise.py**. The exercise.py is running inside the RADI whereas the exercise.html comes from the browser. Both communicate through **websockets**: bidirectional communication channels that allow communication between different programming languages.
Each exercise websocket (typically one for the GUI and one for the robot brain) has its own protocol. The first five characters are used to identify the type of the message.

<a name="Protocol-between-exercise.py-and-browser"></a>
## Protocol between exercise.py and browser
#### **Code websocket**
The code websocket is used to **interact with the brain of the robot**, user's source code is sent from the browser to the exercise.py through this websocket. It has a **standardized protocol**, therefore in the majority of exercises, the code websocket (on **port 1905**) has these commands available:

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

#### **GUI websocket**
The GUI websocket (on **port 2303**) is commonly used to send data from the backend to the frontend since exercise.py has no graphical interface. Gui websocket **sends images to the browser to be displayed** (e.g: robot camera) and displays them with **html widgets**. This websocket varies according to the requirements of each exercise, it has more **variability**. The data is included on a message starting with “#gui” and has different fields based on the exercise. The most common fields are:

- “#image” with the image obtained from the camera of the robot
- “#map” with the position and rotation of the robot


<a name="Protocol-between-manager.py-and-browser"></a>
## Protocol between manager.py and browser
#### **Manager websocket**
The manager websocket is in charge of **requesting** the exercises and **handling** the control of the simulation (On **port 6080**, listening to VNC server on 5900).. For example, it starts/stop/pause/resume/kill the **Gazebo simulation** of the exercise chosen. It also starts the **VNC server**. The code written by the user is sent first from the browser to the manager.py process through the manager websockets. Then, the manager.py checks the code with Pylint and returns the result to the browser.
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

- **websocket manager**: requests the exercises and controls the simulation
- **websocket code**: interacts with the brain of the robot
- **websocket GUI**: receives data from the robot
- **GZClient VNC**: interacts and visualize the simulation
- **Console VNC**: displays debug and print messages

![robotics academy architecture image](/docs/images/robotics_academy_architecture.png "Robotics Academy Architecture")

<a name="User-code-processing"></a>
## User code processing
When a user requests to load the code in the robot, the code follows these steps:
1. The code is **sent from** the **ACE Editor** of the browser **to** the **manager.py** process of the RADI. The code is **checked** by Pylint and the **errors** are returned to the browser. If the browser receives an error, the error is displayed on a modal and the code is not sent to the brain.
2. If there aren't any errors, the code is **sent from** the **browser to** the **exercise.py** through the code websocket. Exercise.py receives the user's source code as raw text and puts it to work.
3. The exercise.py **separates** the **code** in two portions: the **sequential part** (executed once) and the **iterative part** (executed every brain interval). The code is separated by the first while True loop encountered. Within the iterative part the **brain measures** the **time after each iteration**, this is called **code management**, so as not to saturate the CPU and keep a **controlled rhythm** of iterations per second to leave the CPU free for other browser tasks. The iterative part is inserted into another template along with extra code that controls the iterations per second that are carried out, (computational skeleton) so this **computational engine** is tied to the user code to ensure that the code is executed at a **nominal frequency**.
The user code is also enriched with some execution control elements in order to pause, reset and load a new code into the robot brain.

<a name="Threads-used-in-the-exercises"></a>
## Threads used in the exercises 

- **Thread**: Name of the thread
- **Location**: Function in which we can find the thread 
- **Description**: Brief description of thread's function
- **Target**: The target variable sets the function that each thread will execute
- **Arguments**: Arguments used in the execution of the function 

### Exercise.py 

| Thread                 | Location        | Description                                                                           | Target          | Arguments |
|------------------------|-----------------|---------------------------------------------------------------------------------------|-----------------|-----------|
| `self.measured_thread` | `connected()`   | It measures the frequency of iterations                                | `self.measure_frequency `      | None  |
| `self.stats_thread`    | `connected()`   | Tracks the real time factor from Gazebo statistics (not found in all exercises)   | `self.track_stats`      | None  |
| `self.thread`          | `execute_thread()`| Redirects the information to console and runs sequential and iterative code (takes care of the compute engine) | `self.process_code` | source_code  |

### gui.py

| Thread                 | Location                    | Description                                                               | Target          | Arguments |
|------------------------|-----------------------------|---------------------------------------------------------------------------|-----------------|-----------|
| `t`                    | class GUI `execute_thread()`| It activates the server                                                   | `self.run_server` | None  |
| `self.measured_thread` | class ThreadGUI `start()`   | Measuring thread to measure frequency.                                    | `self.measure_thread `      | None  |
| `self.thread`          | class ThreadGUI `start()`   | The main thread of execution.                                             | `self.run`      | None  |

<a name="Flow-Control"></a>
## Flow Control
In order to control the number of messages sent by the users, so the RADI is not overflown with them, both the manager websocket and the code websocket have response messages for certain orders which are sent after the operetions are completed.

- The manager websocket responds with "PingDone" after the operations "start", "stop" or "reset" are completed.
- The code websocket responds with "#exec" after the sent code has been loaded in the brain of the robot.

After any of the previous commands is requested by the user, the respective button is blocked until the completed response returns.

<a name="Manager.py extended"></a>
## Manager.py extended

When the RADI container is opened, it starts the execution of the manager.py. Then wait for the user to pick an exercise. Once the exercise is selected the browser communicates to the manager.py the identifier of that exercise and differentes messages depending the user's actions.

Once the manager.py receives the "start" message, it searches for the instructions.json file. Inside that file is a serie of instructions for each exercise in JSON format. This instructions include things as the path to the exercise.py file of each exercise or the route to the file for launching Gazebo, etc.

Once the manager.py has the name of the exercise, it recovers from the instructions.json file the object whose key match the exercise name.

One of the most important parts of the manager.py comes when the user clicks in the laun button. Then the browser sends a "open" message to the manager and it starts all the necessary things to make the exercise work. As there are many different exercises which require different things, depending on the type of the selected one, the manager.py does different things. For example, for the exercises that need a circuit starts the Gazebo server with that circuit. Meanwhile, for the exercises that dont require anything special it simply starts the VNC and the console.

Once the user introduces and sends their code, the manager.py opens a subprocess to check it searching for possible mistakes. If there are any errors, a pop up will be showed indicating the erros found, a code with the type of error it is and the line in which they can be found. In case there is nothing to be changed, the manager.py executes the user code.


The manager is the one in charge of requesting exercises and controlling the simulation. The principal part of the manager can be found in the manager class. In this class values like the server, client, exercise and simulator used are inicializated and changed afterwards depending on the exercise the user is trying to use.

Ones of the most important function in the manager.py script are the following ones:

- **Handle**: The function in charge of handling the messages received from the browser. Depending the value of this message, it executes one function or another. For example, the "evaluate" command triggers the function to evaluate code sent by user. The "stop" command executes the stop_simulation function which stops the physics, etc
- **Open_simulation**: This function starts everything that is necessary to run the selected exercise. Its behaviour depends on the selected exercise as explained before.
- **Reset_simulation**: Its behaviour too depends on the selected exercise. Some of the exercises just require the drone or topics to be reseted after the "reset" message is sent to the manager.py and others may require a hard_reset, starting the exercise from 0.
