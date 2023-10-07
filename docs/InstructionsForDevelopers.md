### [Back to main README.][]

[Back to main README.]: ../README.md

# Instructions for developers
- [Getting started with Robotics Academy for developers](https://youtu.be/3AM-ztcRsr4) 
- [How to setup the developer environment](#How-to-setup-the-developer-environment)
- [How to add a new exercise](#How-to-add-a-new-exercise)
- [How to update static files version](#How-to-update-static-files-version)
- [Steps to change models from CustomRobots in RoboticsAcademy exercises](#Steps-to-change-models-from-CustomRobots-in-RoboticsAcademy-exercises)
- [How to create a React based exercise](#How-to-create-a-React-based-exercise)
- [Guidelines to render a React based exercise](#Guidelines-to-render-a-React-based-exercise)

<a name="How-to-setup-the-developer-environment"></a>
## How to setup the developer environment 
To get started with developing exercise in Robotics Academy, a developers needs to follow these steps:

prerequisite - Python

1) First create a virtual env
```
virtualenv env_name
 ```
Virtual environment with name "env_name" is created 

2) Activate the environment
```
source env_name/bin/activate
```

3) Confirm that the env is successfully selected
```
which python3
```

4) Install required packages
```
pip install django
pip install djangorestframework
pip install django-webpack-loader
pip install django-cors-headers
pip install pylint==2.* # afaik not working for v3.*, tested on 2.17.4
```

5) Install node in venv
```
pip install nodeenv
nodeenv -p
npm install -g npm  # check installation
npm -v
```

6) Install dependencies for REACT (with Yarn or npm, required Node.JS >= 14.16)     
```
cd react_frontend/ && yarn install && yarn run dev
```

7) Now at the root of the project we are ready to launch the Django webserver
```
python3 manage.py runserver
```
The webserver is not connected with the RADI.

8) To connect the webserver with RADI, Run:
```
docker run --rm -it -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 -p 7163:7163 jderobot/robotics-academy --no-server
```

<a name="How-to-add-a-new-exercise"></a>
## How to add a new exercise
To include a new exercise, add the folder with the exercise contents in exercises/static/exercises following the file name conventions:
- ```entry_point/ros_version```: used for the entrypoint of an exercise run by the RAM
- ```launch/ros_version```: used for world launch files (.launch)
- ```python_template/ros_version```: used for the python templates needed to compose the user code
- ```react-components```: exercise specific react components

Then, create the entry in db.sqlite3. A simple way to do this is by using the Django admin page:
1)  Run ```python3.8 manage.py runserver```.
2)  Access http://127.0.0.1:7164/admin/ on a browser and log in with "user" and "pass".
3)  Click on "add exercise" and fill the required fields specified below. Save and exit.
4)  Commit db.sqlite3 changes.

An exercise entry in the database must include the following data:
- ```exercise id```: unique exercise identifier, must match the folder name
- ```name```: name to display on the exercise list
- ```description```: description to display on the exercise list
- ```tags```: an exercise must include at least one ROS tag ("ROS1" or "ROS2"). The exercise will only be shown on the exercise list when the RADI ROS version installed is listed in the tags. Tags are also used by the search bar.
- ```state```: changes the state indicator (active = green; prototype = yellow; inactive = red)
- ```language```: programming language used
- ```configuration```: available launch options to run the exercise written in JSON. If the generic react components are used, the exercise frontend will automatically request to launch the exercise using the first configuration that matches the key ROSX (X = ROS version detected by django). If the generic circuit selector react component is used, it will automatically display all the launch options items of the array that matches the key ROSX (X = ROS version detected by django), displaying the name stored under the key "name". Sample configuration JSON including 2 launch options for ROS1 and 1 launch option for ROS2:
```
{"ROS1":[
{
  "application": {
    "type": "python",
    "entry_point": "$EXERCISE_FOLDER/entry_point/exercise.py",
    "params": { "circuit": "default"}

  },
  "launch": {
    "0": {
      "type": "module",
      "module": "ros_api",
      "resource_folders": [
        "$EXERCISE_FOLDER/launch/ros1_noetic"
      ],
      "model_folders": [
        "$CUSTOM_ROBOTS_FOLDER/f1/models"
      ],
      "plugin_folders": [
      ],
      "parameters": [],
      "launch_file": "$EXERCISE_FOLDER/launch/ros1_noetic/simple_line_follower_ros_headless_default.launch",
      "name": "Default"
    },
    "1": {
      "type": "module",
      "module": "console",
      "display": ":1",
      "internal_port": 5901,
      "external_port": 1108
    },
    "2": {
      "type": "module",
      "module": "gazebo_view",
      "display": ":0",
      "internal_port": 5900,
      "external_port": 6080,
      "height": 768,
      "width": 1024
    }
  }
},
{
  "application": {
    "type": "python",
    "entry_point": "$EXERCISE_FOLDER/entry_point/exercise.py",
    "params": { "circuit": "default"}

  },
  "launch": {
    "0": {
      "type": "module",
      "module": "ros_api",
      "resource_folders": [
        "$EXERCISE_FOLDER/launch/ros1_noetic"
      ],
      "model_folders": [
        "$CUSTOM_ROBOTS_FOLDER/f1/models"
      ],
      "plugin_folders": [
      ],
      "parameters": [],
      "launch_file": "$EXERCISE_FOLDER/launch/ros1_noetic/simple_line_follower_ros_headless_nbg.launch",
      "name": "Nürburgring"
    },
    "1": {
      "type": "module",
      "module": "console",
      "display": ":1",
      "internal_port": 5901,
      "external_port": 1108
    },
    "2": {
      "type": "module",
      "module": "gazebo_view",
      "display": ":0",
      "internal_port": 5900,
      "external_port": 6080,
      "height": 768,
      "width": 1024
    }
  }
}],
"ROS2":
[
{
  "application": {
    "type": "python",
    "entry_point": "$EXERCISE_FOLDER/entry_point/ros2_humble/exercise.py",
    "params": { "circuit": "default"}
  },
  "launch": {
    "0": {
      "type": "module",
      "module": "ros2_api",
      "resource_folders": [
        "$EXERCISE_FOLDER/launch/ros2_humble"
      ],
      "model_folders": [
        "$CUSTOM_ROBOTS_FOLDER/f1/models"
      ],
      "plugin_folders": [
      ],
      "parameters": [],      
      "launch_file": "$EXERCISE_FOLDER/launch/ros2_humble/simple_line_follower_default.launch.py",
      "name": "Default"

    },
    "1": {
      "type": "module",
      "module": "console_ros2",
      "display": ":1",
      "internal_port": 5901,
      "external_port": 1108
    },
    "2": {
      "type": "module",
      "module": "gazebo_view_ros2",
      "display": ":0",
      "internal_port": 5900,
      "external_port": 6080,
      "height": 768,
      "width": 1024
    }
  }
}
]
}
```

<a name="How-to-update-static-files-version"></a>
## How to update static files version
Follow this steps after changing any js or css document in order to prevent the browser cache to be used:

1º Make all the changes necesary to the required documents.

2º When the changes are done and ready to commit, open settings.py (located on ```RoboticsAcademy/academy/settings.py```).

3º In ```setting.py```, update VERSION with the current date (the format is DD/MM/YYYY so for example the date 17/06/2021 would look something like this ```VERSION = 17062021``` ).

4º Save and commit the changes.

If a new static file is created or you find a file that doesn't have (or updates) their version number, just add ```?v={{SYS_VERSION}}``` to the end of the src.

For example: ```script src="{% static 'exercises/assets/js/utils.js``` would have his src update as follows: ```script src="{% static 'exercises/assets/js/utils.js?v={{SYS_VERSION}}' %}"```

<a name="Steps-to-change-models-from-CustomRobots-in-RoboticsAcademy-exercises"></a>
## Steps to change models from CustomRobots in RoboticsAcademy exercises.
- Upload the new model files to [CustomRobots repository](https://github.com/JdeRobot/CustomRobots)
- Change the model name in .world file contained in static/exercises path that calls it.
- If you have some .world files you need to create different .launch files and add a '{}' in the [instructions.json file](https://github.com/JdeRobot/RoboticsAcademy/blob/master/scripts/instructions.json) that will be replaced by the [manager.py file](https://github.com/JdeRobot/RoboticsAcademy/blob/master/scripts/manager.py) for the variable name of the selection list of the JS and HTML files of the exercise.
- You need to change the launcher.js file in the case that the exercise has a map selector or not.
- Finally, if the exercise need an specific plugin that isn't installed in the container you need to modify the [Dockerfile](https://github.com/JdeRobot/RoboticsAcademy/blob/master/scripts/Dockerfile) an add the commands that allows the installation of the .cc and .hh files of the CustomRobots repository.

## Edit code on RADI On The GO.

1. If your IDE of choice is VSCode then this method is for you, visit [Remote Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) to download the extenstion.

2. Start the Robotics Academy Docker Image

3. Start VS Code

4. Run the Remote-Containers: Open Folder in Container... command and select the local folder.
   
   <img width="597" alt="remote-command-palette" src="https://user-images.githubusercontent.com/58532023/184609609-eb1c1a15-9666-46f9-bc9d-df099d3738b8.png">

## How to add your local changes to RADI while persisting changes two-way

1. This method is for you if you have worked your way till now in your local setup and looking to import all changes inside RADI while also being able to edit and persist further changes.

2. On Terminal open the directory where your project or code is located at (Example:- ```cd ~/my_project```)

3. Append ```-v $(pwd):/location_in_radi``` to your ```docker run``` cli command used to run your container. (Example:- ```docker run --rm -it -p 7164:7164 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 -v $(pwd):/home jderobot/robotics-academy```)

4. This will import your local directory inside the docker container, if you have used the example command like above where the location the command is being run is mounted to the home folder inside the docker container you will simply be able to see all the local mounted directories inside the /home of the RADI.

5. To make sure that your local directory has been mounted correctly to the correct location inside RADI, navigate to http://localhost:1108/vnc.html after launching an exercise(This involves clicking on the launch button of any exercise of your choice) and this will open an vnc console Instance where you may verify the integrity of the mount.

   ![Screenshot from 2022-08-22 01-31-16](https://user-images.githubusercontent.com/58532023/185808802-3a207cb5-b2df-466f-a7f1-70864ff34206.png)

<a name="How-to-create-a-React-based-exercise"></a>
## How to create a React based exercise

All the components build in React are present inside the "react_frontend/src/components".
1. Create the Main Exercise Component follow as -

Add Exercise theory url inside the "react_frontend/src/helpers/TheoryUrlGetter.js"
```angular2html
<Box>
      <ViewProvider>
        <ExerciseProvider>
          <MainAppBar />
          <View url={THEORY_URL."exercise_id"} exercise="{<ExerciseIdView />}" />
        </ExerciseProvider>
      </ViewProvider>
</Box>
```

The **View Component**  handles the different views - theory, exercise and code view.
2. Create an Exercise View Component which contains all the components required in an exercise.

<a name="Guidelines-to-render-a-React-based-exercise"></a>
## Guidelines to render a React based exercise 

1. Create a folder with the folder name as "exercise_id" at the location from repository root : "exercises/static/exercises"
     
This folder contains the static files related to the exercise, it consists of python scripts handling the GUI, Brain and exercise.
2. Create a folder with the folder name as "exercise_id" at the location from repository root : "exercises/templates/exercises"

This folder contains exercise.html which serves React from Django server with the help of tag "react_component". For example -
```angular2html
    {% react_component components/exercises/"exercise_id" %} 
     // Here you can add Child Components [ React Component or HTML ]
    {% end_react_component %}
```
```bash
├── react_frontend
│   ├── src
│       ├── components
│           ├── exercises 
├── exercises
│   ├── static
│   ├── templates  
└── 
```
| Relative path | Absolute path |
| ------------- | ------------- |
| components/exercises/3DReconstructionReact | react_frontend/src/components/exercises/3DReconstructionReact |

Make sure to use the relative path while rendering the component
3. Follow the steps to [add a new exercise in Django ](#How-to-add-a-new-exercise)

