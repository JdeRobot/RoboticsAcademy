#!/bin/sh

curl -sL https://raw.githubusercontent.com/JdeRobot/RoboticsAcademy/superthin-dev/docker-compose-user.yml -o docker-compose.yml && docker-compose up; docker-compose down
