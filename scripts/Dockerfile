FROM jderobot/robotics-academy:rotors_test


# # Custom Robot Repository
# RUN mkdir -p /opt/jderobot && cd /opt/jderobot && \
# 	git clone -b $ROS_DISTRO-devel https://github.com/JdeRobot/CustomRobots.git


# RUN pip install django-webpack-loader djangorestframework

# # bootstrap rosdep
# RUN rosdep init && rosdep update

# # jderobot_drones src and stdr simulator
# RUN add-apt-repository ppa:rock-core/qt4 -y && \
#     apt update --fix-missing
# RUN rm -rf /catkin_ws && \
#     mkdir -p /catkin_ws/src && \
#     cd /catkin_ws && \
#     catkin init && \
#     cd /catkin_ws/src && \
#     git clone https://github.com/RUFFY-369/CrazyS.git && \
#     git clone https://github.com/RUFFY-369/mav_comm.git && \
#     cd /catkin_ws
# COPY .rosinstall /tmp/
# RUN vcs import /catkin_ws/src  < /tmp/.rosinstall


# #CrazyS
# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# RUN apt install -y curl
# RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
# RUN apt update
# RUN apt install -y ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink
# RUN apt install -y ros-noetic-octomap-mapping ros-noetic-control-toolbox
# RUN apt install -y python3-vcstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev
# # RUN rosdep init
# RUN rosdep update
# RUN cd /catkin_ws/src 
# # RUN catkin_init_workspace
# RUN cd /catkin_ws && rosdep install --from-paths src -y -i
# RUN rosdep update
# RUN catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False

# RUN cd /catkin_ws && rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO
# RUN cd /catkin_ws && rosdep update && rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
# RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd /catkin_ws; catkin build -j1'
RUN ["cp", "/catkin_ws/build/rotors_gazebo_plugins/Actuators.pb.cc", "/catkin_ws/src/CrazyS/rotors_gazebo_plugins/src/"] && \
    ["cp", "/catkin_ws/build/rotors_gazebo_plugins/Actuators.pb.h", "/catkin_ws/src/CrazyS/rotors_gazebo_plugins/src/"] && \
    ["cp", "/catkin_ws/build/rotors_gazebo_plugins/libmav_msgs.so", "/catkin_ws/devel/lib/"]

CMD /bin/bash -c 'source /catkin_ws/devel/setup.bash'



# RoboticsAcademy
RUN rm -rf /RoboticsAcademy && \
    git clone https://github.com/RUFFY-369/RoboticsAcademy.git -b rotors
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