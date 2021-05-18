FROM jderobot/base

# Get the Custom Robot Repository
# and set the environment variables
RUN mkdir -p /opt/jderobot && cd /opt/jderobot && git clone https://github.com/JdeRobot/CustomRobots.git \
    && cd CustomRobots && git checkout melodic-devel && git pull origin melodic-devel

## Setting env vars
ENV GAZEBO_RESOURCE_PATH "$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-9:/RoboticsAcademy/exercises/follow_line/web-template/launch"
ENV GAZEBO_PLUGIN_PATH "$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/Firmware/build/px4_sitl_default/build_gazebo"
ENV GAZEBO_MODEL_PATH "$GAZEBO_MODEL_PATH:/usr/share/gazebo-9/models:/opt/jderobot/CustomRobots/f1:/opt/jderobot/CustomRobots/roomba_robot:/opt/jderobot/CustomRobots/3d_reconstruction:/opt/jderobot/CustomRobots/Taxi_navigator:/Firmware/Tools/sitl_gazebo/models:/opt/ros/melodic/share/drone_assets/models:/opt/ros/melodic/share/drone_assets/urdf"
ENV LD_LIBRARY_PATH "$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/Firmware/build/px4_sitl_default/build_gazebo"

# GzWeb
# https://answers.gazebosim.org//question/15807/node-gyp-build-errors-while-trying-to-install-gzweb/
RUN curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
RUN apt-get install -y nodejs
RUN git clone https://github.com/osrf/gzweb.git
RUN cd gzweb && git checkout gzweb_1.4.0 && npm run deploy --- -m local

# VNC
RUN apt-get -y install x11vnc

RUN git clone https://github.com/novnc/noVNC.git &&\
	cd noVNC/utils &&\
	git checkout tags/v1.2.0 &&\ 
	git clone https://github.com/novnc/websockify websockify &&\
	cd websockify &&\
	git checkout tags/v0.9.0

# Console
RUN apt-get -y install xterm

# RoboticsAcademy
RUN git clone https://github.com/JdeRobot/RoboticsAcademy.git && cd RoboticsAcademy && git pull origin master
RUN rsync -a --exclude 'ace-builds' /RoboticsAcademy/exercises/static/exercises/* /RoboticsAcademy/exercises


# Certain Dependencies for Exercises
# Vacuum Cleaner Exercise
RUN apt-get -y --quiet install ros-melodic-kobuki-msgs

# Drone-cat-mouse
RUN apt-get -y --quiet --no-install-recommends install xmlstarlet

# Follow Turtlebot exercise
RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
	ros-$ROS_DISTRO-turtlebot3-description



# Put the other commands in bashrc file
RUN echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc
RUN echo 'source /usr/share/gazebo-9/setup.sh' >> ~/.bashrc

# Install and run XDummy XServer
RUN apt-get update && apt-get -y install xserver-xorg-video-dummy x11-apps
RUN wget https://xpra.org/xorg.conf
RUN Xorg -noreset +extension GLX +extension RANDR +extension RENDER -logfile ./xdummy.log -config ./xorg.conf :0 &
RUN echo 'export DISPLAY=:0' >> ~/.bashrc

RUN echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/Firmware' >> ~/.bashrc
RUN echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/Firmware/Tools/sitl_gazebo' >> ~/.bashrc

# websocket server dependency
RUN apt-get update && apt-get install -y python2.7 python-pip
RUN pip install websocket_server posix-ipc
COPY manager.py /manager.py
RUN apt-get install -y python3.8 python3-pip
RUN python3.8 -m pip install websockets asyncio

LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

# django to serve the exercises
RUN pip3 install django

# execute permissions to start.sh
COPY start.sh /start.sh
RUN chmod 755 /start.sh

EXPOSE 2303
EXPOSE 1905
EXPOSE 8080
EXPOSE 7681
EXPOSE 8765
EXPOSE 1108
# Port by WebRtc
EXPOSE 1831
# Port for noVNC
EXPOSE 6080