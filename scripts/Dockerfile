FROM jderobot/base

# Custom Robot Repository
RUN mkdir -p /opt/jderobot && cd /opt/jderobot && \ 
	git clone -b melodic-devel https://github.com/JdeRobot/CustomRobots.git && \
	git clone https://github.com/JdeRobot/IndustrialRobots.git

## Setting env vars
ENV GAZEBO_RESOURCE_PATH "$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-9:/RoboticsAcademy/exercises/follow_line/web-template/launch"
ENV GAZEBO_PLUGIN_PATH "$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/Firmware/build/px4_sitl_default/build_gazebo"
ENV GAZEBO_MODEL_PATH "$GAZEBO_MODEL_PATH:/usr/share/gazebo-9/models:/opt/jderobot/CustomRobots/f1:/opt/jderobot/CustomRobots/roomba_robot:/opt/jderobot/CustomRobots/3d_reconstruction:/opt/jderobot/CustomRobots/Taxi_navigator:/opt/jderobot/IndustrialRobots/assets/models:/Firmware/Tools/sitl_gazebo/models:/opt/ros/melodic/share/drone_assets/models:/opt/ros/melodic/share/drone_assets/urdf"
ENV LD_LIBRARY_PATH "$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/Firmware/build/px4_sitl_default/build_gazebo"
ENV ROS_PACKAGE_PATH "$ROS_PACKAGE_PATH:/opt/jderobot/IndustrialRobots/industrial_robots"

# RoboticsAcademy
RUN git clone https://github.com/predator4hack/RoboticsAcademy.git -b pick_place
# RUN rsync -a --exclude 'ace-builds' /RoboticsAcademy/exercises/static/exercises/* /RoboticsAcademy/exercises

COPY manager.py /manager.py
COPY instructions.json /instructions.json

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