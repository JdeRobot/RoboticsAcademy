### [Back to main README.][]

[Back to main README.]: ../README.md

# Developers info about repository architecture
- [master branch](#master-branch)
- [gh-pages branch](#gh-pages-branch)
- [issues branches](#issues-branches)
- [Other repositories](#Other-repositories)

<a name="master-branch"></a>
## master branch

[Master branch](https://github.com/JdeRobot/RoboticsAcademy/tree/master) of the RoboticsAcademy repository is divided in some folders that contains different types of codes. There are 4 main folders: docs, exercises, static and scripts.

- **docs** folder holds all documentation about the repository and its architecture.
- **exercises** folder contains all the codes related to the exercises launch process and visualization. In it you can find:
	1. HTML codes of every exercise (exercise.html) --> (/exercises/templates/exercises).
	2. Base HTML file (exercise_base.html) with the exercise view navbars and buttons, and modal files with the pop-up messages --> (/exercises/templates).
	3. Python and User Interface (UI) codes used in the exercises (/exercises/static).
- **static** folder has all resources used by the codes in the exercises folder such as images and javascript and css files called by HTML codes. This folder is divided in:
	1. Common folder, that holds all shared resources between the different exercises (common images, javascript, css...) --> (/static/common).
	2. Exercise folder, that holds all specific resources that only a single exercise use that file. It's divided in folders with the exercise name --> (/static/exercises).
- **scripts** folder, that hosts the dockerfile (file with the Docker commands to create a RADI), shell files, manager.py (file used to manage the exercises processes) and pyint_checker.py (file used to check if the code has been written properly).

<a name="gh-pages-branch"></a>
## gh-pages branch

The [gh-pages branch](https://github.com/JdeRobot/RoboticsAcademy/tree/gh-pages) contains part of the source code of the front-end. It's separated into some folders that holds html, json, xml and markdown files. Main folders are:

- **_pages**: this folder stores all markdown files that are imported to other html files. in this folder you can find the exercise folder in which the markdown corresponding to the documentation of the various exercises of the repository can be found.
	1. Autonomous Cars: text documentation of the exercises 'autoparking', 'car-junction', 'follow_line', 'global_navigation' and 'obstacle_avoidance'.
	2. Computer Vision: text documentation of the exercises '3d_reconstruction', 'color_filter', 'follow_face', 'human_detection', 'montercarlo_visual_loc', 'opticalflow_teleop' and 'visual_odometry'.
	3. Drones: text documentation of the exercises 'drone_cat_mouse', 'drone_gymkhana', 'drone_hangar', 'follow_road', 'follow_turtlebot', 'labyrinth_escape', 'package_delivery', 'position_control', 'rescue_people' and 'visual_lander'.
	4. IndustrialRobots: text documentation of the exercises 'machine_vision', 'mobile_manipulation' and 'pick_place'.
	5. MobileRobots: text documentation of the exercises 'amazon_warehouse', 'bump_and_go', 'laser_mapping', 'localization_laser', 'multi_robot_amazon_warehouse', 'vacuum_cleaner' and 'vacuum_cleaner_loc'.
- **assets**: this folder contains all css, js and images resources used by the front-end documentation pages.
- **_includes**: this folder has all html files that are used to structure the webpage front-end (head, footer, search bar...) and some other resources such as the youtubePlayer.html file. This resources are called by the Jekyll template through the tag {%include xxxxx.html %}. You can obtain more information [**here**](https://jekyllrb.com/docs/includes/) .
- **_layouts**: this folder stores the html example templates used to create the webpage html files.

<a name="issues-branches"></a>
## issues branches

The rest of the branches will have the designation 'issue-xxxx', being xxxx the name correspondent to the incidence they are attached to. This number is given automatically by GitHub when an issue is open.


<a name="Other-repositories"></a>
## Other repositories
Robotics Academy includes two JdeRobot repositories as dependencies.
- [CustomRobots](https://github.com/JdeRobot/CustomRobots) contains different types of robots (vehicles, service robots...) and world files for Gazebo.
- [drones](https://github.com/JdeRobot/drones) contains different types of drones and world files for Gazebo.