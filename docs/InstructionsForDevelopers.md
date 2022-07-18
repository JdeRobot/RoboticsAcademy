### [Back to main README.][]

[Back to main README.]: ../README.md

# Instructions for developers
- [How to setup the developer environment](#How-to-setup-the-developer-environment)
- [How to add a new exercise](#How-to-add-a-new-exercise)
- [How to update static files version](#How-to-update-static-files-version)
- [Steps to change models from CustomRobots in RoboticsAcademy exercises](#Steps-to-change-models-from-CustomRobots-in-RoboticsAcademy-exercises)

<a name="How-to-setup-the-developer-environment"></a>
## How to setup the developer environment 
To get started with developing exercise in Robotics Academy, a developers needs to follow these steps:

prerequisite - Python
1) First create a virtual env
```
virtualenv env
 ```
Virtual environment with name "env" is created 
2) Activate the environment
```
source env/bin/activate
```
3) Install required packages
```
pip install django
pip install djangorestframework
pip install django-webpack-loader
```

4) Now we are ready to launch the Django webserver
```
python3 manage.py runserver
```
The webserver is not connected with the RADI.
5) To connect the webserver with RADI, Run:
```
docker run --rm -it -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy —no-server
```

<a name="How-to-add-a-new-exercise"></a>
## How to add a new exercise
To include a new exercise, add the folder with the exercise contents in exercises/static/exercises following the file name conventions. Then, create the entry in db.sqlite3. A simple way to do this is by using the Django admin page:
1)  Run ```python3.8 manage.py runserver```.
2)  Access http://127.0.0.1:8000/admin/ on a browser and log in with "user" and "pass".
3)  Click on "add exercise" and fill the fields: exercise id (folder name), name (name to display), state, language and description (description to display). Save and exit.
4)  Commit db.sqlite3 changes.

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
