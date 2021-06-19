<a href="https://mmg-ai.com/en/"><img src="https://jderobot.github.io/assets/images/logo.png" width="100 " align="right" /></a>

# RoboticsAcademy: Learn Robotics, Artificial Intelligence and Computer Vision 


JdeRobot Academy is an **open source** collection of exercises to learn robotics in a practical way. Gazebo simulator is the main tool required, as ROS. Its latest documentation (including installation recipes, current available exercises and illustrative videos) is on its <a href="https://jderobot.github.io/RoboticsAcademy">webpage</a>.

If you are a contributor, please note that we use GitHub Pages and a Jekyll theme (MinimalMistakes) for Academy web page. Feel free to install Jekyll locally, so that, you can test your changes before submitting your pull-request.

## How to contribute?

Take a look at the [contribute section](https://jderobot.github.io/RoboticsAcademy/contribute/) to join this project.

## For developers

To include a new exercise, add the folder with the exercise contents in exercises/static/exercises following the file name conventions. Then, create the entry in db.sqlite3. A simple way to do this is by using the Django admin page:
1)  Run ```python3.8 manage.py runserver```.
2)  Access http://127.0.0.1:8000/admin/ on a browser and log in with "user" and "pass".
3)  Click on "add exercise" and fill the fields: exercise id (folder name), name (name to display), state, language and description (description to display). Save and exit.
4)  Commit db.sqlite3 changes.


### How to update static files version number

Follow this steps when changing any js or css document:

1º Make all the changes necesary to the required documents.

2º When the changes are done and ready to commit, open settings.py (located on ```RoboticsAcademy/academy/settings.py```).

3º In ```setting.py```, update VERSION with the current date (the format is DD/MM/YYYY so for example the date 17/06/2021 would look something like this ```VERSION = 17062021``` ).

4º Save and commit the changes.

If a new static file is created or you find a file that doesn't have (or updates) their version number, just add ```?v={{SYS_VERSION}}``` to the end of the src.

For example: ```script src="{% static 'exercises/assets/js/utils.js``` would have his src update as follows: ```script src="{% static 'exercises/assets/js/utils.js?v={{SYS_VERSION}}' %}"```
