<a href="https://mmg-ai.com/en/"><img src="https://jderobot.github.io/assets/images/logo.png" width="100 " align="right" /></a>

# RoboticsAcademy: Learn Robotics, Artificial Intelligence and Computer Vision 


JdeRobot Academy is an **open source** collection of exercises to learn robotics in a practical way. Gazebo simulator is the main tool required, as ROS. Its latest documentation (including installation recipes, current available exercises and illustrative videos) is on its <a href="https://jderobot.github.io/RoboticsAcademy">webpage</a>.

If you are a contributor, please note that we use GitHub Pages and a Jekyll theme (MinimalMistakes) for Academy web page. Feel free to install Jekyll locally, so that, you can test your changes before submitting your pull-request.

## How to contribute?

Take a look at the [contribute section](https://jderobot.github.io/RoboticsAcademy/contribute/) to join this project.

# Instructions for developers

## How to add a new exercise
To include a new exercise, add the folder with the exercise contents in exercises/static/exercises following the file name conventions. Then, create the entry in db.sqlite3. A simple way to do this is by using the Django admin page:
1)  Run ```python3.8 manage.py runserver```.
2)  Access http://127.0.0.1:8000/admin/ on a browser and log in with "user" and "pass".
3)  Click on "add exercise" and fill the fields: exercise id (folder name), name (name to display), state, language and description (description to display). Save and exit.
4)  Commit db.sqlite3 changes.


## How to update static files version 
Follow this steps after changing any js or css document in order to prevent the browser cache to be used:

1º Make all the changes necesary to the required documents.

2º When the changes are done and ready to commit, open settings.py (located on ```RoboticsAcademy/academy/settings.py```).

3º In ```setting.py```, update VERSION with the current date (the format is DD/MM/YYYY so for example the date 17/06/2021 would look something like this ```VERSION = 17062021``` ).

4º Save and commit the changes.

If a new static file is created or you find a file that doesn't have (or updates) their version number, just add ```?v={{SYS_VERSION}}``` to the end of the src.

For example: ```script src="{% static 'exercises/assets/js/utils.js``` would have his src update as follows: ```script src="{% static 'exercises/assets/js/utils.js?v={{SYS_VERSION}}' %}"```

# Robotics Academy Architecture
## Robotics Academy frontend
Robotics frontend is served from a Django webserver running inside the RADI. Each exercise page communicates with different elements of the RADI in order to interact with the simulation.

## Robotics Academy backend
The Robotics Academy Docker Image (RADI) has several exercises available using Gazebo and STDR. In order to request and interact with the exercises, the container has a websocket port (8765) and a communication protocol (Robotics Academy Manager Protocol, or RAMP). Each exercise opens 1, 2 or more websockets to interact specifically with the exercise and receive data. The RAMP includes these commands:

    “open” in order to start an exercise specified on the field “exercise”
    “stop” to stop the simulation
    “resume” to resume the simulation
    “reset” to reset the simulation
    “evaluate” to request an evaluation of the code sent on the field “code”
    “startgz” to open the viewer GZClient
    “stopgz” to close the viewer GZClient
    “Ping” or "PingDone" to send Ping messages and to communicate that an order has been executed (ater resume, reset or stop commands)

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

#### **GUI websocket**
The GUI websocket (on port 2303) is commonly used to send data from the backend to the frontend. The data is included on a message starting with “#gui” and has different fields based on the exercise. The most common fields are:

- “#image” with the image obtained from the camera of the robot
- “#map” with the position and rotation of the robot

## Frontend-backend communication
The connection between the backend and the frontend consists of these elements:

- websocket manager: requests the exercises and controls the simulation
- websocket code: interacts with the brain of the robot
- websocket GUI: receives data from the robot
- GZClient VNC: interacts and visualize the simulation
- Console VNC: displays debug and print messages

![robotics academy architecture image](/docs/images/robotics_academy_architecture.png "Robotics Academy Architecture")

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

## Other repositories
Robotics Academy includes two JdeRobot repositories as dependencies.
- [CustomRobots](https://github.com/JdeRobot/CustomRobots) contains different types of robots (vehicles, service robots...) and world files for Gazebo.
- [drones](https://github.com/JdeRobot/drones) contains differnt types of drones and world files for Gazebo.

## Developers info about repository architecture

### master branch

Master branch of the RoboticsAcademy repository is divided in some folders that contains different types of codes. There are 4 main folders: docs, exercises, static and scripts.

- **docs** folder holds all documentation about the repository and its architecture.
- **exercises** folder contains all the codes related to the exercises launch process and visualization. In it you can find:
	1. HTML codes of every exercise (exercise.html) --> (/exercises/templates/exercises).
	2. Base HTML file (exercise_base.html) with the exercise view navbars and buttons, and modal files with the pop-up messages --> (/exercises/templates).
	3. Python and User Interface (UI) codes used in the exercises (/exercises/static).
- **static** folder has all resources used by the codes in the exercises folder such as images and javascript and css files called by HTML codes. This folder is divided in:
	1. Common folder, that holds all shared resources between the different exercises (common images, javascript, css...) --> (/static/common).
	2. Exercise folder, that holds all specific resources that only a single exercise use that file. It's divided in folders with the exercise name --> (/static/exercises).
- **scripts** folder, that hosts the dockerfile (file with the Docker commands to create a RADI), shell files, manager.py (file used to manage the exercises processes) and pyint_checker.py (file used to check if the code has been written properly).

### gh-pages branch

The gh-pages branch contains part of the source code of the front-end. It's separated into some folders that holds html, json, xml and markdown files. Main folders are:

- **_pages**: this folder stores all markdown files that are imported to other html files. in this folder you can find the exercise folder in which the markdown corresponding to the documentation of the various exercises of the repository can be found.
	1. Autonomous Cars: text documentation of the exercises 'autoparking', 'car-junction', 'follow_line', 'global_navigation' and 'obstacle_avoidance'.
	2. Computer Vision: text documentation of the exercises '3d_reconstruction', 'color_filter', 'follow_face', 'human_detection', 'montercarlo_visual_loc', 'opticalflow_teleop' and 'visual_odometry'.
	3. Drones: text documentation of the exercises 'drone_cat_mouse', 'drone_gymkhana', 'drone_hangar', 'follow_road', 'follow_turtlebot', 'labyrinth_escape', 'package_delivery', 'position_control', 'rescue_people' and 'visual_lander'.
	4. IndustrialRobots: text documentation of the exercises 'machine_vision', 'mobile_manipulation' and 'pick_place'.
	5. MobileRobots: text documentation of the exercises 'amazon_warehouse', 'bump_and_go', 'laser_mapping', 'localization_laser', 'multi_robot_amazon_warehouse', 'vacuum_cleaner' and 'vacuum_cleaner_loc'.
- **assets**: this folder contains all css, js and images resources used by the front-end documentation pages.
- **_includes**: this folder has all html files that are used to structure the webpage front-end (head, footer, search bar...) and some other resources such as the youtubePlayer.html file.
- **_layouts**: this folder stores the html example templates used to create the webpage html files.