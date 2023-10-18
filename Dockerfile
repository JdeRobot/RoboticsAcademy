FROM jderobot/robotics-academy:latest

# Clone the RoboticsApplicationManager repository into the src folder inside RoboticsAcademy
ARG RAM=main
RUN rm -rf RoboticsApplicationManager
RUN git clone https://github.com/JdeRobot/RoboticsApplicationManager.git -b $RAM /RoboticsApplicationManager

# RoboticsAcademy
ARG ROBOTICS_ACADEMY=exercise/visual_odometry_3D/dev
RUN rm -rf RoboticsAcademy
RUN git clone --depth 10 https://github.com/JdeRobot/RoboticsAcademy.git -b $ROBOTICS_ACADEMY

# Relocate RAM
RUN mkdir /RoboticsAcademy/src/ && mv /RoboticsApplicationManager/* /RoboticsAcademy/src

# React
RUN npm install -g yarn
RUN cd /RoboticsAcademy/react_frontend && yarn install && yarn run build

EXPOSE 8000
EXPOSE 2303
EXPOSE 1905
EXPOSE 8765
EXPOSE 6080
EXPOSE 1108
EXPOSE 7163
EXPOSE 7164

ENTRYPOINT [ "./entrypoint.sh" ]


# # Clone the RoboticsApplicationManager repository into the src folder inside RoboticsAcademy
# ARG RAM=main
# RUN git clone https://github.com/JdeRobot/RoboticsApplicationManager.git -b $RAM /RoboticsApplicationManager

# # RoboticsAcademy
# ARG ROBOTICS_ACADEMY=master
# RUN git clone --depth 10 https://github.com/JdeRobot/RoboticsAcademy.git -b $ROBOTICS_ACADEMY

# # Relocate RAM
# RUN mkdir /RoboticsAcademy/src/ && mv /RoboticsApplicationManager/* /RoboticsAcademy/src

# # React
# RUN npm install -g yarn
# RUN cd /RoboticsAcademy/react_frontend && yarn install && yarn run build

# # Scripts copy
# RUN mv -t ./ /opt/jderobot/scripts/.env /opt/jderobot/scripts/instructions.json /opt/jderobot/scripts/pylint_checker.py /opt/jderobot/scripts/entrypoint.sh
# RUN mv /opt/jderobot/scripts/pylintrc /etc/pylintrc

# RUN rm -rf /usr/bin/python /usr/bin/python2 /usr/bin/python2.7 \
#     && ln -s /usr/bin/python3.8 /usr/bin/python

# # SSH for development
# RUN apt-get update && apt-get install -y openssh-server
# RUN mkdir /var/run/sshd
# RUN echo 'root:password' | chpasswd
# RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
# RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd
# CMD ["/usr/sbin/sshd","-D"]

# EXPOSE 22

# # Django server
# EXPOSE 7164

# # Manager websocket
# EXPOSE 8765

# # Code websocket
# EXPOSE 1905
# EXPOSE 1904

# # GUI websockets
# EXPOSE 2303
# EXPOSE 2304

# # noVNC Console
# EXPOSE 1108

# # noVNC Gazebo
# EXPOSE 6080

# # WebRtc
# EXPOSE 1831

# # RAM
# EXPOSE 7163

# ENTRYPOINT [ "./entrypoint.sh" ]