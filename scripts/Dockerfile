FROM osrf/ros:melodic-desktop-full
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}

ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update && apt-get install -y xvfb libjansson-dev libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential

# Get the Custom Robot Repository
# and set the environment variables
RUN mkdir -p /opt/jderobot && cd /opt/jderobot && git clone https://github.com/JdeRobot/CustomRobots.git && cd CustomRobots && git checkout melodic-devel && git pull origin melodic-devel
ENV GAZEBO_MODEL_PATH=/opt/jderobot/CustomRobots/f1:/opt/jderobot/CustomRobots/roomba_robot

# GzWeb
# https://answers.gazebosim.org//question/15807/node-gyp-build-errors-while-trying-to-install-gzweb/
RUN curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
RUN apt-get install -y nodejs
RUN git clone https://github.com/osrf/gzweb.git
RUN cd gzweb && git checkout gzweb_1.4.0 && npm run deploy --- -m local

# RoboticsAcademy
RUN git clone https://github.com/JdeRobot/RoboticsAcademy.git && cd RoboticsAcademy && git pull origin master
ENV GAZEBO_RESOURCE_PATH=/RoboticsAcademy/exercises/follow_line/web-template/launch

# Certain Dependencies for Exercises
# Vacuum Cleaner Exercise
RUN apt-get install ros-melodic-kobuki-msgs         

# Put the other commands in bashrc file
RUN echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc
RUN echo 'source /usr/share/gazebo-9/setup.sh' >> ~/.bashrc
RUN echo 'export Xvfb :0 -screen 0 1024x768x16 &> /dev/null' >> ~/.bashrc
RUN echo 'export DISPLAY=:0' >> ~/.bashrc

# websocket server dependency
RUN apt-get update && apt-get install -y python2.7 python-pip
RUN pip install websocket_server
COPY manager.py /manager.py
RUN apt-get install -y python3.8 python3-pip
RUN python3.8 -m pip install websockets asyncio

EXPOSE 2303
EXPOSE 1905
EXPOSE 8080
EXPOSE 7681
EXPOSE 8765

