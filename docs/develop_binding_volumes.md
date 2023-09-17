# Develop using volume binding
A fast way of development and testing can be achieved using volume binding. This allows you to run a RADI including changes from your local file system without building the image. This method is only suitable when the changes are made on the repositories RoboticsAcademy or RoboticsApplicationManager. When modifying the dependencies on RoboticsInfrastructure, a new image should be built.

## Prerequisites
- Developer environment installed. Follow the instructions on [How to set up the developer environment section.][]

## Instructions
This method consists in replicating the folder /RoboticsAcademy that exists inside the RADI on your local drive. Then, a docker run option is used to replace the RADI folder with your local folder. By doing so, all the changes made on your local files are included in the container. Note that this is a shared volume, so all changes made to your shared volume even from inside the container are persistent.
1. Replicate the folder RADI:/RoboticsAcademy inside your local file system. This can be achieved by cloning the repository [RoboticsAcademy][https://github.com/JdeRobot/RoboticsAcademy] and the repository [RoboticsApplicationManager][https://github.com/JdeRobot/RoboticsApplicationManager] inside the folder RoboticsAcademy/src (include the contents directly, not into a subfolder src/RoboticsApplicationManager).
2. Run ``` yarn run dev ``` if there were any front-end changes
3. Run the docker image mounting the volume with the instruction ``` docker run --rm -it --name radi -p 7164:7164 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 -p 7163:7163 --mount type=bind,source="$(pwd)"/RoboticsAcademy,target=/RoboticsAcademy jderobot/robotics-academy ``` replacing the mount source path to your RoboticsAcademy path.


[How to set up the developer environment section.]: /docs/InstructionsForDevelopers.md