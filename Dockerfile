FROM jderobot/robotics-academy:latest

ARG CACHEBUST=0

RUN cd / 

# RoboticsAcademy
RUN rm -rf RoboticsAcademy/
RUN git clone https://github.com/JdeRobot/RoboticsAcademy.git -b exercise/visual_odometry_3D/dev

# instructions.json
RUN rm instructions.json
RUN mv RoboticsAcademy/instructions.json /

EXPOSE 8000
EXPOSE 2303
EXPOSE 1905
EXPOSE 8765
EXPOSE 6080
EXPOSE 1108
EXPOSE 7163

ENTRYPOINT [ "./entrypoint.sh" ]
