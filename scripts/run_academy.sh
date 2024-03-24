#!/bin/sh

curl -sL https://raw.githubusercontent.com/JdeRobot/RoboticsAcademy/master/docker-compose-cpu.yaml -o docker-compose.yml && docker-compose up; docker-compose down; rm docker-compose.yml
