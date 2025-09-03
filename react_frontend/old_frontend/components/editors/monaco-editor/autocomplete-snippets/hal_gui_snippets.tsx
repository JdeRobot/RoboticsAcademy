export interface SnippetItem {
  type: string;
  label: string;
  code: string;
  descriptions: string;
}

export interface SnippetGroup {
  hal: SnippetItem[];
  webgui: SnippetItem[];
}

export interface SnippetCollection {
  [key: string]: SnippetGroup;
}

// HAL & GUI Auto Complete Objects
export const guiAndHalAutoCompleteObj: SnippetCollection = {
  _follow_line: {
    hal: [
      {
        type: "method",
        label: "getImage()",
        code: "getImage()",
        descriptions: "Get the image (BGR8).",
      },
      {
        type: "method",
        label: "setV(velocity)",
        code: "setV(velocity)",
        descriptions: "Set the linear speed.",
      },
      {
        type: "method",
        label: "setW(velocity)",
        code: "setW(velocity)",
        descriptions: "Set the angular velocity.",
      },
    ],
    webgui: [
      {
        type: "method",
        label: "showImage(image)",
        code: "showImage(image)",
        descriptions:
          "Allows you to view a debug image or with relevant information.",
      },
    ],
  },
  _vacuum_cleaner: {
    hal: [
      {
        type: "method",
        label: "getBumperData().state",
        code: "getBumperData().state",
        descriptions:
          "To establish if the robot has crashed or not. Returns a 1 if the robot collides and a 0 if it has not crashed.",
      },
      {
        type: "method",
        label: "getBumperData().bumper",
        code: "getBumperData().bumper",
        descriptions:
          "If the robot has crashed, it turns to 1 when the crash occurs at the center of the robot, 0 when it occurs at its right and 2 if the collision is at its left.",
      },
      {
        type: "method",
        label: "getLaserData()",
        code: "getLaserData()",
        descriptions:
          "It allows to obtain the data of the laser sensor, which consists of 180 pairs of values ​​(0-180º, distance in meters).",
      },
      {
        type: "method",
        label: "setV()",
        code: "setV()",
        descriptions: "Set the linear speed).",
      },
      {
        type: "method",
        label: "setW()",
        code: "setW()",
        descriptions: "Set the angular velocity",
      },
    ],
    webgui: [],
  },
  _autoparking: {
    hal: [
      {
        type: "method",
        label: "getPose3d()",
        code: "getPose3d()",
        descriptions: "Get all the position information.",
      },
      {
        type: "method",
        label: "getPose3d().x",
        code: "getPose3d().x",
        descriptions: "Get the position of the robot (x coordinate).",
      },
      {
        type: "method",
        label: "getPose3d().y",
        code: "getPose3d().y",
        descriptions: "Get the position of the robot (y coordinate).",
      },
      {
        type: "method",
        label: "getPose3d().yaw",
        code: "getPose3d().yaw",
        descriptions:
          "Get the orientation of the robot with regarding the map.",
      },
      {
        type: "method",
        label: "getFrontLaserData()",
        code: "getFrontLaserData()",
        descriptions:
          "Obtain the front laser sensor data It is composed of 180 pairs of values: (0-180º distance in millimeters).",
      },
      {
        type: "method",
        label: "getRightLaserData()",
        code: "getRightLaserData()",
        descriptions:
          "Obtain the right laser sensor data It is composed of 180 pairs of values: (0-180º distance in millimeters).",
      },
      {
        type: "method",
        label: "getBackLaserData()",
        code: "getBackLaserData()",
        descriptions:
          "Obtain the back laser sensor data It is composed of 180 pairs of values: (0-180º distance in millimeters).",
      },
      {
        type: "method",
        label: "setV()",
        code: "setV()",
        descriptions: "Set the linear speed.",
      },
      {
        type: "method",
        label: "setW()",
        code: "setW()",
        descriptions: "Set the angular velocity.",
      },
    ],
    webgui: [],
  },
  _follow_person: {
    hal: [
      {
        type: "method",
        label: "getImage()",
        code: "getImage()",
        descriptions: "Obtain the current frame of the camera robot.",
      },
      {
        type: "method",
        label: "getPose3d().x",
        code: "getPose3d().x",
        descriptions: "Get the position of the robot (x coordinate).",
      },
      {
        type: "method",
        label: "getPose3d().y",
        code: "getPose3d().y",
        descriptions: "Obtain the position of the robot (and coordinate).",
      },
      {
        type: "method",
        label: "getPose3d().yaw",
        code: "getPose3d().yaw",
        descriptions:
          "Get the orientation of the robot with respect to the map.",
      },
      {
        type: "method",
        label: "getLaserData()",
        code: "getLaserData()",
        descriptions:
          "It allows to obtain the data of the laser sensor. It returns a list of 180 laser measurements (0 - 180 degrees).",
      },
      {
        type: "method",
        label: "setV()",
        code: "setV()",
        descriptions: "Set the linear speed.",
      },
      {
        type: "method",
        label: "setW()",
        code: "setW()",
        descriptions: "Set the angular velocity.",
      },
      {
        type: "method",
        label: "getBoundingBoxes()",
        code: "getBoundingBoxes()",
        descriptions:
          "This method calls a detect() neural network's method to obtain a list of detected objects from an image passed as argument.",
      },
    ],
    webgui: [
      {
        type: "method",
        label: "showImage()",
        code: "showImage()",
        descriptions: "Show an opencv image in the web template.",
      },
    ],
  },
  _vacuum_cleaner_loc: {
    hal: [
      {
        type: "method",
        label: "setV()",
        code: "setV()",
        descriptions: "Set the linear speed.",
      },
      {
        type: "method",
        label: "setW()",
        code: "setW()",
        descriptions: "Set the angular velocity.",
      },
      {
        type: "method",
        label: "getPose3d().x",
        code: "getPose3d().x",
        descriptions: "Get the X coordinate of the robot.",
      },
      {
        type: "method",
        label: "getPose3d().y",
        code: "getPose3d().y",
        descriptions: "Get the Y coordinate of the robot.",
      },
      {
        type: "method",
        label: "getPose3d().yaw",
        code: "getPose3d().yaw",
        descriptions: "Get the orientation of the robot.",
      },
      {
        type: "method",
        label: "getBumperData().state",
        code: "getBumperData().state",
        descriptions:
          "To establish if the robot has crashed or not. Returns a 1 if the robot collides and a 0 if it has not crashed.",
      },
      {
        type: "method",
        label: "getBumperData().bumper",
        code: "getBumperData().bumper",
        descriptions:
          "If the robot has crashed, it turns to 1 when the crash occurs at the center of the robot, 0 when it occurs at its right and 2 if the collision is at its left.",
      },
      {
        type: "method",
        label: "getLaserData()",
        code: "getLaserData()",
        descriptions:
          "It allows to obtain the data of the laser sensor, which consists of 180 pairs of values ​​(0-180º, distance in meters).",
      },
    ],
    webgui: [
      {
        type: "method",
        label: "showNumpy(mat)",
        code: "showNumpy(mat)",
        descriptions: "Displays the matrix sent.",
      },
      {
        type: "method",
        label: "getMap(url)",
        code: "getMap(url)",
        descriptions:
          "Returns a numpy array with the image data in a 3 dimensional array (R, G, B, A). The URL of the Vacuum Cleaner.",
      },
    ],
  },
  _global_navigation: {
    hal: [
      {
        type: "method",
        label: "setV()",
        code: "setV()",
        descriptions: "Set the linear speed.",
      },
      {
        type: "method",
        label: "setW()",
        code: "setW()",
        descriptions: "Set the angular velocity.",
      },
      {
        type: "method",
        label: "getPose3d()",
        code: "getPose3d()",
        descriptions:
          "Returns x,y and theta components of the robot in world coordinates.",
      },
    ],
    webgui: [
      {
        type: "method",
        label: "showNumpy(numpy)",
        code: "showNumpy(numpy)",
        descriptions:
          "Shows Gradient Path Planning field on the user interface. It represents the values of the field that have been assigned to the array passed as a parameter. Accepts as input a two-dimensional uint8 numpy array whose values can range from 0 to 255 (grayscale). In order to have a grid with the same resolution as the map, the array should be 400x400.",
      },
      {
        type: "method",
        label: "showPath(array)",
        code: "showPath(array)",
        descriptions:
          "Shows a path on the map. The parameter should be a 2D array containing each of the points of the path.",
      },
      {
        type: "method",
        label: "getTargetPose()",
        code: "getTargetPose()",
        descriptions:
          "Returns x,y coordinates of chosen destionation in the world.",
      },
      {
        type: "method",
        label: "getMap(url)",
        code: "getMap(url)",
        descriptions:
          "Returns a numpy array with the image data in grayscale as a 2 dimensional array.",
      },
      {
        type: "method",
        label: "rowColumn(vector)",
        code: "rowColumn(vector)",
        descriptions:
          "Returns the index in map coordinates corresponding to the vector in world coordinates passed as parameter.",
      },
    ],
  },
  _rescue_people: {
    hal: [
      {
        type: "method",
        label: "get_position()",
        code: "get_position()",
        descriptions:
          "Returns the actual position of the drone as a numpy array [x, y, z], in m.",
      },
      {
        type: "method",
        label: "get_velocity()",
        code: "get_velocity()",
        descriptions:
          "Returns the actual velocities of the drone as a numpy array [vx, vy, vz], in m/s.",
      },
      {
        type: "method",
        label: "get_yaw_rate()",
        code: "get_yaw_rate()",
        descriptions: "Returns the actual yaw rate of the drone, in rad/s.",
      },
      {
        type: "method",
        label: "get_orientation()",
        code: "get_orientation()",
        descriptions:
          "Returns the actual roll, pitch and yaw of the drone as a numpy array [roll, pitch, yaw], in rad.",
      },
      {
        type: "method",
        label: "get_roll()",
        code: "get_roll()",
        descriptions: "Returns the roll angle of the drone, in rad.",
      },
      {
        type: "method",
        label: "get_pitch()",
        code: "get_pitch()",
        descriptions: "Returns the pitch angle of the drone, in rad.",
      },
      {
        type: "method",
        label: "get_yaw()",
        code: "get_yaw()",
        descriptions: "Returns the yaw angle of the drone, in rad.",
      },
      {
        type: "method",
        label: "get_landed_state()",
        code: "get_landed_state()",
        descriptions:
          "Returns 1 if the drone is on the ground (landed), 2 if the drone is in the air and 4 if the drone is landing. 0 could be also returned if the drone landed state is unknown.",
      },
      {
        type: "method",
        label: "set_cmd_pos(x, y, z, az)",
        code: "set_cmd_pos(x, y, z, az)",
        descriptions:
          "Commands the position (x,y,z) of the drone, in m and the yaw angle (az) (in rad) taking as reference the first takeoff point (map frame).",
      },
      {
        type: "method",
        label: "set_cmd_vel(vx, vy, vz, az)",
        code: "set_cmd_vel(vx, vy, vz, az)",
        descriptions:
          "Commands the linear velocity of the drone in the x, y and z directions (in m/s) and the yaw rate (az) (rad/s) in its body fixed frame.",
      },
      {
        type: "method",
        label: "set_cmd_mix(vx, vy, z, az)",
        code: "set_cmd_mix(vx, vy, z, az)",
        descriptions:
          "Commands the linear velocity of the drone in the x, y directions (in m/s), the height (z) related to the takeoff point and the yaw rate (az) (in rad/s)",
      },
      {
        type: "method",
        label: "takeoff(height)",
        code: "takeoff(height)",
        descriptions:
          "Takeoff at the current location, to the given height (in m).",
      },
      {
        type: "method",
        label: "land()",
        code: "land()",
        descriptions: "Land at the current location.",
      },
      {
        type: "method",
        label: "get_frontal_image()",
        code: "get_frontal_image()",
        descriptions:
          "Returns the latest image from the frontal camera as a OpenCV cv2_image.",
      },
      {
        type: "method",
        label: "get_ventral_image()",
        code: "get_ventral_image()",
        descriptions:
          "Returns the latest image from the ventral camera as a OpenCV cv2_image.",
      },
    ],
    webgui: [
      {
        type: "method",
        label: "showImage(cv2_image)",
        code: "showImage(cv2_image)",
        descriptions: "Shows a image of the camera in the GUI.",
      },
      {
        type: "method",
        label: "showLeftImage(cv2_image)",
        code: "showLeftImage(cv2_image)",
        descriptions: "Shows another image of the camera in the GUI.",
      },
    ],
  },
  _obstacle_avoidance: {
    hal: [
      {
        type: "method",
        label: "getPose3d().x",
        code: "getPose3d().x",
        descriptions: "Get the position of the robot (x coordinate).",
      },
      {
        type: "method",
        label: "getPose3d().y",
        code: "getPose3d().y",
        descriptions: "Obtain the position of the robot (y coordinate).",
      },
      {
        type: "method",
        label: "getPose3d().yaw",
        code: "getPose3d().yaw",
        descriptions:
          "Get the orientation of the robot with regarding the map.",
      },
      {
        type: "method",
        label: "getLaserData()",
        code: "getLaserData()",
        descriptions:
          "Obtain laser sensor data It is composed of 180 pairs of values: (0-180º distance in meters).",
      },
      {
        type: "method",
        label: "setV()",
        code: "setV()",
        descriptions: "Set the linear speed.",
      },
      {
        type: "method",
        label: "setW()",
        code: "setW()",
        descriptions: "Set the angular velocity.",
      },
    ],
    webgui: [
      {
        type: "method",
        label: "getNextTarget()",
        code: "getNextTarget()",
        descriptions: "Obtain the next target object on the scenario.",
      },
      {
        type: "method",
        label: "setTargetx",
        code: "setTargetx",
        descriptions: "Sets the x coordinate of the target on the GUI.",
      },
      {
        type: "method",
        label: "setTargety",
        code: "setTargety",
        descriptions: "Sets the y coordinate of the target on the GUI.",
      },
      {
        type: "method",
        label: "showForces",
        code: "showForces",
        descriptions:
          "Shows the forces being appliend on the car in real time.",
      },
    ],
  },
  _3d_reconstruction: {
    hal: [
      {
        type: "method",
        label: "getImage('left')",
        code: "getImage('left')",
        descriptions: "Get the left image.",
      },
      {
        type: "method",
        label: "getImage('right')",
        code: "getImage('right')",
        descriptions: "Get the right image.",
      },
      {
        type: "method",
        label: "getCameraPosition('left')",
        code: "getCameraPosition('left')",
        descriptions: "Get the left camera position from ROS Driver Camera.",
      },
      {
        type: "method",
        label: "getCameraPosition('right')",
        code: "getCameraPosition('right')",
        descriptions: "Get the right camera position from ROS Driver Camera.",
      },
      {
        type: "method",
        label: "graficToOptical('left', point2d)",
        code: "graficToOptical('left', point2d)",
        descriptions:
          "Transform the Image Coordinate System to the Camera System.",
      },
      {
        type: "method",
        label: "backproject('left', point2d)",
        code: "backproject('left', point2d)",
        descriptions: "Backprojects the 2D Image Point into 3D Point Space.",
      },
      {
        type: "method",
        label: "project('left', point3d)",
        code: "project('left', point3d)",
        descriptions: "Backprojects a 3D Point Space into the 2D Image Point.",
      },
      {
        type: "method",
        label: "opticalToGrafic('left', point2d)",
        code: "opticalToGrafic('left', point2d)",
        descriptions:
          "Transform the Camera System to the Image Coordinate System.",
      },
      {
        type: "method",
        label: "project3DScene(point3d)",
        code: "project3DScene(point3d)",
        descriptions:
          "Transform 3D Point Space after triangulation to the 3D Point Viewer.",
      },
    ],
    webgui: [
      {
        type: "method",
        label: "ShowNewPoints(points)",
        code: "ShowNewPoints(points)",
        descriptions: "Plot a array of plots in the 3D visor.",
      },
      {
        type: "method",
        label: "ShowAllPoints(points)",
        code: "ShowAllPoints(points)",
        descriptions: "Clear the 3D visor and plot new array of plots.",
      },
      {
        type: "method",
        label: "ClearAllPoints()",
        code: "ClearAllPoints()",
        descriptions: "Clear the 3D visor.",
      },
      {
        type: "method",
        label: "showImageMatching(x1, y1, x2, y2)",
        code: "showImageMatching(x1, y1, x2, y2)",
        descriptions: "Plot the matching between two images.",
      },
      {
        type: "method",
        label: "showImages(imageLeft,imageRight,True)",
        code: "showImages(imageLeft,imageRight,True)",
        descriptions:
          "Allows you to view a debug images or with relevant information.",
      },
    ],
  },
  _amazon_warehouse: {
    hal: [
      {
        type: "method",
        label: "getPose3d()",
        code: "getPose3d()",
        descriptions:
          "Returns x,y and theta components of the robot in world coordinates.",
      },
      {
        type: "method",
        label: "getSimTime()",
        code: "getSimTime()",
        descriptions: "Returns simulation time.",
      },
      {
        type: "method",
        label: "setV()",
        code: "setV()",
        descriptions: "Set the linear speed.",
      },
      {
        type: "method",
        label: "setW()",
        code: "setW()",
        descriptions: "Set the angular speed.",
      },
      {
        type: "method",
        label: "lift()",
        code: "lift()",
        descriptions: "Lift the platform",
      },
      {
        type: "method",
        label: "putdown()",
        code: "putdown()",
        descriptions: "Put down the platform.",
      },
    ],
    webgui: [
      {
        type: "method",
        label: "showPath(array)",
        code: "showPath(array)",
        descriptions:
          "Shows a path on the map. The parameter should be a 2D array containing each of the points of the path.",
      },
      {
        type: "method",
        label: "getMap(url)",
        code: "getMap(url)",
        descriptions:
          "Returns a numpy array with the image data in a 3 dimensional array (R, G, B, A).",
      },
      {
        type: "method",
        label: "showNumpy(mat)",
        code: "showNumpy(mat)",
        descriptions: "Displays the matrix sent.",
      },
    ],
  },
  _montecarlo_laser_loc: {
    hal: [
      {
        type: "method",
        label: "setV()",
        code: "setV()",
        descriptions: "Set the linear speed.",
      },
      {
        type: "method",
        label: "setW()",
        code: "setW()",
        descriptions: "Set the angular velocity.",
      },
      {
        type: "method",
        label: "getPose3d().x",
        code: "getPose3d().x",
        descriptions: "Get the position of the robot (x coordinate).",
      },
      {
        type: "method",
        label: "getPose3d().y",
        code: "getPose3d().y",
        descriptions: "Obtain the position of the robot (y coordinate).",
      },
      {
        type: "method",
        label: "getPose3d().yaw",
        code: "getPose3d().yaw",
        descriptions:
          "Get the orientation of the robot with regarding the map.",
      },
      {
        type: "method",
        label: "getOdom()",
        code: "getOdom()",
        descriptions:
          "Get the orientation of the robot with noise regarding the map.",
      },
      {
        type: "method",
        label: "getLaserData()",
        code: "getLaserData()",
        descriptions:
          "Obtain laser sensor data It is composed of 180 pairs of values: (0-180º distance in meters).",
      },
      {
        type: "method",
        label: "getBumperData()",
        code: "getBumperData()",
        descriptions:
          "If the robot has crashed, it turns to 1 when the crash occurs at the center of the robot, 0 when it occurs at its right and 2 if the collision is at its left.",
      },
    ],
    webgui: [
      {
        type: "method",
        label: "showPosition(x, y, angle)",
        code: "showPosition(x, y, angle)",
        descriptions: "Shows the position on the map.",
      },
      {
        type: "method",
        label: "getMap(url)",
        code: "getMap(url)",
        descriptions:
          "Returns a numpy array with the image data in a 3 dimensional array (R, G, B, A). The URL of the Vacuum Cleaner.",
      },
      {
        type: "method",
        label: "poseToMap(x_prime, y_prime, yaw_prime)",
        code: "poseToMap(x_prime, y_prime, yaw_prime)",
        descriptions: "Returns the map coordinates of the given pose.",
      },
      {
        type: "method",
        label: "mapToPose(x, y, yaw)",
        code: "mapToPose(x, y, yaw)",
        descriptions: "Returns the pose of the given map coordinates.",
      },
      {
        type: "method",
        label: "showParticles(particles)",
        code: "showParticles(particles)",
        descriptions: "Shows the particles on the map.",
      },
    ],
  },
  _montecarlo_visual_loc: {
    hal: [
      {
        type: "method",
        label: "getImage()",
        code: "getImage()",
        descriptions: "Get the image.",
      },
      {
        type: "method",
        label: "setV()",
        code: "setV()",
        descriptions: "Set the linear speed.",
      },
      {
        type: "method",
        label: "setW()",
        code: "setW()",
        descriptions: "Set the angular velocity.",
      },
      {
        type: "method",
        label: "getPose3d().x",
        code: "getPose3d().x",
        descriptions: "Get the position of the robot (x coordinate).",
      },
      {
        type: "method",
        label: "getPose3d().y",
        code: "getPose3d().y",
        descriptions: "Obtain the position of the robot (y coordinate).",
      },
      {
        type: "method",
        label: "getPose3d().yaw",
        code: "getPose3d().yaw",
        descriptions:
          "Get the orientation of the robot with regarding the map.",
      },
    ],
    webgui: [
      {
        type: "method",
        label: "showPosition(x, y, angle)",
        code: "showPosition(x, y, angle)",
        descriptions: "Shows the position on the map.",
      },
      {
        type: "method",
        label: "showImage()",
        code: "showImage()",
        descriptions:
          "Allows you to view a debug image or with relevant information.",
      },
      {
        type: "method",
        label: "getMap(url)",
        code: "getMap(url)",
        descriptions: "Returns the map from the given url.",
      },
      {
        type: "method",
        label: "getBGRMap(url)",
        code: "getBGRMap(url)",
        descriptions: "Returns the map as a cv2 image from the given url.",
      },
      {
        type: "method",
        label: "poseToMap(x_prime, y_prime, yaw_prime)",
        code: "poseToMap(x_prime, y_prime, yaw_prime)",
        descriptions:
          "Converts the robot position to 2D coordinates in the map.",
      },
      {
        type: "method",
        label: "mapToPose(x, y, yaw)",
        code: "mapToPose(x, y, yaw)",
        descriptions:
          "Converts the robot 2D coordinates in the map to the real 3D position.",
      },
      {
        type: "method",
        label: "showParticles(particles)",
        code: "showParticles(particles)",
        descriptions: "Shows the particles on the map.",
      },
    ],
  },
  _marker_visual_loc: {
    hal: [
      {
        type: "method",
        label: "getImage()",
        code: "getImage()",
        descriptions: "Get the image.",
      },
      {
        type: "method",
        label: "setV()",
        code: "setV()",
        descriptions: "Set the linear speed.",
      },
      {
        type: "method",
        label: "setW()",
        code: "setW()",
        descriptions: "Set the angular velocity.",
      },
      {
        type: "method",
        label: "getPose3d().x",
        code: "getPose3d().x",
        descriptions: "Get the position of the robot (x coordinate).",
      },
      {
        type: "method",
        label: "getPose3d().y",
        code: "getPose3d().y",
        descriptions: "Obtain the position of the robot (y coordinate).",
      },
      {
        type: "method",
        label: "getPose3d().yaw",
        code: "getPose3d().yaw",
        descriptions:
          "Get the orientation of the robot with regarding the map.",
      },
    ],
    webgui: [
      {
        type: "method",
        label: "showEstimatedPose((x, y, angle))",
        code: "showEstimatedPose((x, y, angle))",
        descriptions: "Shows the estimated position on the map.",
      },
      {
        type: "method",
        label: "showImage()",
        code: "showImage()",
        descriptions:
          "Allows you to view a debug image or with relevant information.",
      },
    ],
  },
  _laser_mapping: {
    hal: [
      {
        type: "method",
        label: "setV()",
        code: "setV()",
        descriptions: "Set the linear speed.",
      },
      {
        type: "method",
        label: "setW()",
        code: "setW()",
        descriptions: "Set the angular velocity.",
      },
      {
        type: "method",
        label: "getPose3d().x",
        code: "getPose3d().x",
        descriptions: "Get the position of the robot (x coordinate).",
      },
      {
        type: "method",
        label: "getPose3d().y",
        code: "getPose3d().y",
        descriptions: "Obtain the position of the robot (y coordinate).",
      },
      {
        type: "method",
        label: "getPose3d().yaw",
        code: "getPose3d().yaw",
        descriptions:
          "Get the orientation of the robot with regarding the map.",
      },
      {
        type: "method",
        label: "getOdom()",
        code: "getOdom()",
        descriptions: "Obtain the position of the robot with a little noise.",
      },
      {
        type: "method",
        label: "getOdom2()",
        code: "getOdom2()",
        descriptions: "Obtain the position of the robot with noise.",
      },
      {
        type: "method",
        label: "getOdom3()",
        code: "getOdom3()",
        descriptions: "Obtain the position of the robot with a lot of noise.",
      },
    ],
    webgui: [
      {
        type: "method",
        label: "setUserMap(image)",
        code: "setUserMap(image)",
        descriptions:
          "Allows you to view a debug image or with relevant information.",
      },
      {
        type: "method",
        label: "poseToMap(x, y, yaw)",
        code: "poseToMap(x, y, yaw)",
        descriptions:
          "Converts the robot position to 2D coordinates in the map.",
      },
    ],
  },
  _follow_road: {
    hal: [
      {
        type: "method",
        label: "get_position()",
        code: "get_position()",
        descriptions:
          "Returns the actual position of the drone as a numpy array [x, y, z], in m.",
      },
      {
        type: "method",
        label: "get_velocity()",
        code: "get_velocity()",
        descriptions:
          "Returns the actual velocities of the drone as a numpy array [vx, vy, vz], in m/s.",
      },
      {
        type: "method",
        label: "get_yaw_rate()",
        code: "get_yaw_rate()",
        descriptions: "Returns the actual yaw rate of the drone, in rad/s.",
      },
      {
        type: "method",
        label: "get_orientation()",
        code: "get_orientation()",
        descriptions:
          "Returns the actual roll, pitch and yaw of the drone as a numpy array [roll, pitch, yaw], in rad.",
      },
      {
        type: "method",
        label: "get_roll()",
        code: "get_roll()",
        descriptions: "Returns the roll angle of the drone, in rad.",
      },
      {
        type: "method",
        label: "get_pitch()",
        code: "get_pitch()",
        descriptions: "Returns the pitch angle of the drone, in rad.",
      },
      {
        type: "method",
        label: "get_yaw()",
        code: "get_yaw()",
        descriptions: "Returns the yaw angle of the drone, in rad.",
      },
      {
        type: "method",
        label: "get_landed_state()",
        code: "get_landed_state()",
        descriptions:
          "Returns 1 if the drone is on the ground (landed), 2 if the drone is in the air and 4 if the drone is landing. 0 could be also returned if the drone landed state is unknown.",
      },
      {
        type: "method",
        label: "set_cmd_pos(x, y, z, az)",
        code: "set_cmd_pos(x, y, z, az)",
        descriptions:
          "Commands the position (x,y,z) of the drone, in m and the yaw angle (az) (in rad) taking as reference the first takeoff point (map frame).",
      },
      {
        type: "method",
        label: "set_cmd_vel(vx, vy, vz, az)",
        code: "set_cmd_vel(vx, vy, vz, az)",
        descriptions:
          "Commands the linear velocity of the drone in the x, y and z directions (in m/s) and the yaw rate (az) (rad/s) in its body fixed frame.",
      },
      {
        type: "method",
        label: "set_cmd_mix(vx, vy, z, az)",
        code: "set_cmd_mix(vx, vy, z, az)",
        descriptions:
          "Commands the linear velocity of the drone in the x, y directions (in m/s), the height (z) related to the takeoff point and the yaw rate (az) (in rad/s)",
      },
      {
        type: "method",
        label: "takeoff(height)",
        code: "takeoff(height)",
        descriptions:
          "Takeoff at the current location, to the given height (in m).",
      },
      {
        type: "method",
        label: "land()",
        code: "land()",
        descriptions: "Land at the current location.",
      },
      {
        type: "method",
        label: "get_frontal_image()",
        code: "get_frontal_image()",
        descriptions:
          "Returns the latest image from the frontal camera as a OpenCV cv2_image.",
      },
      {
        type: "method",
        label: "get_ventral_image()",
        code: "get_ventral_image()",
        descriptions:
          "Returns the latest image from the ventral camera as a OpenCV cv2_image.",
      },
    ],
    webgui: [
      {
        type: "method",
        label: "showImage(cv2_image)",
        code: "showImage(cv2_image)",
        descriptions: "Shows a image of the camera in the GUI.",
      },
      {
        type: "method",
        label: "showLeftImage(cv2_image)",
        code: "showLeftImage(cv2_image)",
        descriptions: "Shows another image of the camera in the GUI.",
      },
    ],
  },
  _basic_computer_vision: {
    hal: [],
    webgui: [
      {
        type: "method",
        label: "showImage(image)",
        code: "showImage(image)",
        descriptions:
          "Allows you to view a debug image or with relevant information.",
      },
      {
        type: "method",
        label: "getImage()",
        code: "getImage()",
        descriptions:
          "Returns the image from the camera. Can be None.",
      },
    ],
  },
};

export const frequencyAutoCompleteObj: SnippetItem[] = [
  {
    type: "method",
    label: "tick()",
    code: "tick(20)",
    descriptions: "Regulate the execution rate. Defaults to 20 Hz.",
  },
];
