FROM jderobot/base:3.1

# Custom Robot Repository
RUN mkdir -p /opt/jderobot && cd /opt/jderobot && \
	git clone -b $ROS_DISTRO-devel https://github.com/JdeRobot/CustomRobots.git

# Setting environment variables
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc
RUN echo 'source /catkin_ws/devel/setup.bash' >> ~/.bashrc
RUN echo 'source /usr/share/gazebo-11/setup.sh' >> ~/.bashrc
RUN echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/PX4-Autopilot:/PX4-Autopilot/Tools/sitl_gazebo' >> ~/.bashrc

ENV GAZEBO_RESOURCE_PATH "$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-11:/RoboticsAcademy/exercises/follow_line/web-template/launch"
ENV GAZEBO_PLUGIN_PATH "$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/PX4-Autopilot/build/px4_sitl_default/build_gazebo"
ENV GAZEBO_MODEL_PATH "$GAZEBO_MODEL_PATH:/usr/share/gazebo-11/models:/opt/jderobot/CustomRobots/f1/models:/opt/jderobot/CustomRobots/roomba_robot/models:/opt/jderobot/CustomRobots/3d_reconstruction/models:/opt/jderobot/CustomRobots/Taxi_navigator/models:/opt/jderobot/CustomRobots/car_junction/models:/PX4-Autopilot/Tools/sitl_gazebo/models:/opt/ros/$ROS_DISTRO/share/drone_assets/models:/drones/drone_assets/models:/opt/jderobot/CustomRobots/package_delivery/models"
ENV LD_LIBRARY_PATH "$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/PX4-Autopilot/build/px4_sitl_default/build_gazebo"

# RoboticsAcademy
RUN git clone https://github.com/JdeRobot/RoboticsAcademy.git -b master
RUN rsync -a --exclude 'ace-builds' /RoboticsAcademy/exercises/static/exercises/* /RoboticsAcademy/exercises

# STDR files copy
RUN cp /RoboticsAcademy/exercises/static/exercises/laser_mapping/web-template/launch/amigobot_without_gui.launch /catkin_ws/src/stdr_simulator/stdr_launchers/launch/
RUN cp /RoboticsAcademy/exercises/static/exercises/laser_mapping/stdr_files/amigobot.xml /catkin_ws/src/stdr_simulator/stdr_resources/resources/robots/amigobot.xml
RUN cp /RoboticsAcademy/exercises/static/exercises/laser_mapping/stdr_files/sonar_amigobot.xml /catkin_ws/src/stdr_simulator/stdr_resources/resources/range_sensors/
RUN cp /RoboticsAcademy/exercises/static/exercises/laser_mapping/stdr_files/rplidar_a2.xml /catkin_ws/src/stdr_simulator/stdr_resources/resources/laser_sensors/

# Scripts copy
COPY manager.py /manager.py
COPY instructions.json /instructions.json
COPY start.sh /start.sh
COPY start_manager.sh /start_manager.sh
COPY launch_stdr.sh /launch_stdr.sh
COPY start_logs.sh /start_logs.sh
COPY pylint_checker.py /pylint_checker.py
COPY pylintrc /etc/pylintrc
# execute permissions
RUN chmod 755 /start.sh
RUN chmod 755 /start_manager.sh
RUN chmod 755 /launch_stdr.sh
RUN chmod 755 /start_logs.sh

RUN rm -rf /usr/bin/python /usr/bin/python2 /usr/bin/python2.7 \
    && ln -s /usr/bin/python3.8 /usr/bin/python

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
