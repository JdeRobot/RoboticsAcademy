#!/bin/bash

#People faces spawn loc random
Y1=$((RANDOM%(4))).$((RANDOM%999))
x1=$(shuf -i25-45 -n1)
y1=$(shuf -i25-45 -n1)
Y2=$((RANDOM%(4))).$((RANDOM%999))
x2=$(shuf -i25-45 -n1)
y2=$(shuf -i25-45 -n1)
Y3=$((RANDOM%(4))).$((RANDOM%999))
x3=$(shuf -i25-45 -n1)
y3=$(shuf -i25-45 -n1)
Y4=$((RANDOM%(4))).$((RANDOM%999))
x4=$(shuf -i25-45 -n1)
y4=$(shuf -i25-45 -n1)
Y5=$((RANDOM%(4))).$((RANDOM%999))
x5=$(shuf -i25-45 -n1)
y5=$(shuf -i25-45 -n1)
Y6=$((RANDOM%(4))).$((RANDOM%999))
x6=$(shuf -i25-45 -n1)
y6=$(shuf -i25-45 -n1)

/opt/ros/noetic/bin/roslaunch ./RoboticsAcademy/exercises/rescue_people/web-template/launch/rescue_people.launch x1:=$x1 y1:=$y1 Y1:=$Y1 x2:=$x2 y2:=$y2 Y2:=$Y2 x3:=$x3 y3:=$y3 Y3:=$Y3 x4:=$x4 y4:=$y4 Y4:=$Y4 x5:=$x5 y5:=$y5 Y5:=$Y5 x6:=$x6 y6:=$y6 Y6:=$Y6
