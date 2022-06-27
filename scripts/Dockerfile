FROM jderobot/base:3.2

# Custom Robot Repository
RUN mkdir -p /opt/jderobot && cd /opt/jderobot && \
	git clone -b $ROS_DISTRO-devel https://github.com/JdeRobot/CustomRobots.git

# bootstrap rosdep
RUN rosdep init && rosdep update

# jderobot_drones src and stdr simulator
RUN add-apt-repository ppa:rock-core/qt4 -y && \
    apt update --fix-missing
RUN rm -rf /catkin_ws && \
    mkdir -p /catkin_ws/src && \
    cd /catkin_ws && \
    catkin init
COPY .rosinstall /tmp/
RUN vcs import /catkin_ws/src  < /tmp/.rosinstall

RUN cd /catkin_ws && rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO
RUN cd /catkin_ws && rosdep update && rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd /catkin_ws; catkin build'

# RoboticsAcademy
RUN git clone https://github.com/JdeRobot/RoboticsAcademy.git -b master
RUN rsync -a --exclude 'ace-builds' /RoboticsAcademy/exercises/static/exercises/* /RoboticsAcademy/exercises

# React
RUN cd /RoboticsAcademy/react_frontend && npm install && npm run build && cd

# Scripts copy
COPY [".env", "manager.py", "instructions.json", "pylint_checker.py", "entrypoint.sh", "./"]
COPY pylintrc /etc/pylintrc

RUN rm -rf /usr/bin/python /usr/bin/python2 /usr/bin/python2.7 \
    && ln -s /usr/bin/python3.8 /usr/bin/python

# Django server
EXPOSE 8000

# Manager websocket
EXPOSE 8765

# Code websocket
EXPOSE 1905
EXPOSE 1904

# GUI websockets
EXPOSE 2303
EXPOSE 2304

# noVNC Console
EXPOSE 1108

# noVNC Gazebo
EXPOSE 6080

# WebRtc
EXPOSE 1831


ENTRYPOINT [ "./entrypoint.sh" ]
# CMD ["--help"]