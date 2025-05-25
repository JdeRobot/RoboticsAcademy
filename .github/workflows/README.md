# Actions

When publishing a new version the release workflow will trigger and will publish the new version of the robotics-academy docker and the robotics-database docker images.

## Github Action _Release Robotics Academy_

This action should be used when releasing a new RoboticsAcademy version and when the automatic one failed to do so on the new version release. **You should not use this action when creating a beta or testing image**. For that purpose use _Generate Robotics Academy_ or _Generate Robotics Academy Database_.

This github action is prepared to call the actions _Generate Robotics Academy_ and _Generate Robotics Academy Database_. This github action needs some inputs:

* Docker Image tag: The id of the tag of the RoboticsAcademy image to pull from Dockerhub. It must be the version number __(ex: 5.4.1)__.
* Upload as latest: The id of the tag of the RoboticsAcademy image is also set to latest to pull from Dockerhub. Uncheck only if testing the action.
* Branch of Robotics Academy: The branch of RoboticsAcademy repo that will be packaged. It must be _humble-devel_.
* Branch of RoboticsInfrastructure: The branch of RoboticsInfrastructure repo that will be packaged. It's default value is _humble-devel_.
* Branch of RoboticsInfrastructure for Universes Database: The branch of RoboticsInfrastructure repo where the universes database is stored. It's default value is _database_.
* Branch of RoboticsApplicationManager: The branch of RoboticsApplicationManager repo that will be packaged. It's default value is _humble-devel_.
* ROS Distro: ROS version that will be used on the simulations. From now on RA and Unibotics-webserver only use __ROS 2 (humble)__.

## Github Action _Generate Robotics Academy_

This github action is prepared to package every image created by the Dockerfiles into a RoboticsAcademy tagged docker image. This github action needs some inputs:

* Docker Image tag: The id of the tag of the RoboticsAcademy image to pull from Dockerhub. It is usually a the version number __(ex: 5.4.1)__ unless it's a beta image, which should be tagged as __beta__
* Upload as latest: The id of the tag of the RoboticsAcademy image is also set to latest to pull from Dockerhub. Check only if the image is desired to be used as the latest reference.
* Branch of Robotics Academy: The branch of RoboticsAcademy repo that will be packaged. It's default value is _humble-devel_. Note that If you're creating a beta image of RA, you must specify the branch of RA which has the desired changes.
* Branch of RoboticsInfrastructure: The branch of RoboticsInfrastructure repo that will be packaged. It's default value is _humble-devel_.
* Branch of RoboticsApplicationManager: The branch of RoboticsApplicationManager repo that will be packaged. It's default value is _humble-devel_.
* ROS Distro: ROS version that will be used on the simulations. From now on RA and Unibotics-webserver only use __ROS 2 (humble)__.

  ### How does this github action works?

  1. All the necessary setups for github action is prepared.
  2. The github action logs in the Dockerhub webpage using the __github actions secrets__.
  3. Then builds a base image with all the deppendencies used on robotics applications.
  4. Finally starts compiling and packaging every repo to create the RA image.
  5. New RA image is pushed into [Dockerhub](https://hub.docker.com/r/jderobot/robotics-academy).

## Github Action _Generate Robotics Academy Database_

This github action is prepared to package every image created by the Dockerfiles into a Robotics Academy Database tagged docker image. This github action needs some inputs:

* Docker Image tag: The id of the tag of the Robotics Academy Database image to pull from Dockerhub. It is usually a the version number __(ex: 5.4.1)__ unless it's a beta image, which should be tagged as __beta__
* Upload as latest: The id of the tag of the Robotics Academy Database image is also set to latest to pull from Dockerhub. Check only if the image is desired to be used as the latest reference.
* Branch of RoboticsAcademy: The branch of RoboticsAcademy repo that will be used to retrieve the exercise database and the django_auth databse. It's default value is _humble-devel_. Note that If you're creating a beta image of RA, you must specify the branch of RoboticsAcademy which has the desired changes.
* Branch of RoboticsInfrastructure: The branch of RoboticsInfrastructure repo where the universe database is stored. It's default value is _database_.

  ### How does this github action works?

  1. All the necessary setups for github action is prepared.
  2. The github action logs in the Dockerhub webpage using the __github actions secrets__.
  3. Then builds a base image with all the deppendencies used on robotics applications.
  4. Finally starts compiling and packaging every repo to create the Robotics Academy Database image.
  5. New Bt Studio image is pushed into [Dockerhub](https://hub.docker.com/r/jderobot/robotics-database).