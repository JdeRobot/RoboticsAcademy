export const resizeList = ["min", "max"];
export const editorList = ["ace", "monaco"];
export const monacoEditorThemeList = ["vs", "vs-dark", "hc-black"];
export const defaultEditorSourceCode = `import GUI
import HAL
import numpy as np
# Enter sequential code!

while True:
    # Enter iterative code!
    print("Robotics Academy!")

`;

// Know about pylint Errors with type
// https://gist.github.com/codezerro/f7f696702ee7ea12782d9af3f9bb4f4c
// pylint disable error
export const pylint_error = ["E0401", "E1101"];
export const pylint_warning = ["W0611"];
export const pylint_convention = ["C0114", "C0303", "C0304", "C0305", "C0411"];
export const pylint_refactor = [];
export const pylint_fatal = [];

// python packages autocomplete list

export const listed_python_packages = ["numpy", "math", "cv2", "HAL", "GUI"];

// HAL & GUI Auto Complete Objects
export const guiAndHalAutoCompleteObj = {
  _follow_line_newmanager: {
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
    gui: [
      {
        type: "method",
        label: "showImage(image)",
        code: "showImage(image)",
        descriptions:
          "Allows you to view a debug image or with relevant information.",
      },
    ],
  },
  _vacuum_cleaner_newmanager: {
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
    gui: [],
  },
  _autoparking_newmanager: {
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
    gui: [],
  },
  _follow_person_newmanager: {
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
    gui: [
      {
        type: "method",
        label: "showImage()",
        code: "showImage()",
        descriptions: "Show an opencv image in the web template.",
      },
    ],
  },
  _vacuum_cleaner_loc_newmanager: {
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
    gui: [
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
  _global_navigation_newmanager: {
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
    gui: [
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
  _rescue_people_newmanager: {
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
    gui: [
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
  _obstacle_avoidance_newmanager: {
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
    gui: [
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
  _3d_reconstruction_newmanager: {
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
    gui: [
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
  _amazon_warehouse_newmanager: {
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
    gui: [
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
    ],
  },
  _montecarlo_laser_loc: {
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
    gui: [
      {
        type: "method",
        label: "showImage()",
        code: "showImage()",
        descriptions:
          "Allows you to view a debug image or with relevant information.",
      },
      {
        type: "method",
        label: "showParticles(particles)",
        code: "showParticles(particles)",
        descriptions: "Shows the particles on the map.",
      },
    ],
  },
};

export const getAllSnippets = ({ monaco, range }) => {
  return [
    // RA
    {
      label: "import numpy as np",
      kind: monaco.languages.CompletionItemKind.Class,
      insertText: "import numpy as np",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "import numpy as np",
    },
    {
      label: "import numpy",
      kind: monaco.languages.CompletionItemKind.Class,
      insertText: "import numpy",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "import numpy",
    },
    {
      label: "import math",
      kind: monaco.languages.CompletionItemKind.Class,
      insertText: "import math",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "import math",
    },
    {
      label: "import cv2",
      kind: monaco.languages.CompletionItemKind.Class,
      insertText: "import cv2",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "import cv2",
    },
    // template
    {
      label: "RA: Template-01",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: `import GUI
import HAL
# Enter sequential code!

while True:
    # Enter iterative code!`,
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: `import GUI
import HAL
# Enter sequential code!

while True:
    # Enter iterative code!`,
    },
    // python snippets
    {
      label: "def",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "def ${1:function_name}(${2:args}):\n\t${3:pass}",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "Function Definition",
      range: range,
    },
    {
      label: "if",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ["if ${1:condition}:", "\t${2:# code}"].join("\n"),
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "If Statement",
      range: range,
    },
    {
      label: "elif",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ["elif ${1:condition}:", "\t${2:# code}"].join("\n"),
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "Elif Statement",
      range: range,
    },
    {
      label: "else",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ["else:", "\t${1:# code}"].join("\n"),
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "Else Statement",
      range: range,
    },
    {
      label: "for",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ["for ${1:item} in ${2:iterable}:", "\t${3:# code}"].join(
        "\n"
      ),
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "For Loop",
      range: range,
    },
    {
      label: "while",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ["while ${1:condition}:", "\t${2:# code}"].join("\n"),
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "While Loop",
      range: range,
    },
    {
      label: "try",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: [
        "try:",
        "\t# comment:",
        "except Exception as e:",
        "\traise e",
        "# end try",
      ].join("\n"),
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "While Loop",
      range: range,
    },
    {
      label: "print",
      kind: monaco.languages.CompletionItemKind.Function,
      insertText: "print(${1:message})",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: {
        value:
          "Defines a Python function named `fun`.\n\nSnippet for a simple function definition.",
      },
    },
    {
      label: "return",
      kind: monaco.languages.CompletionItemKind.Keyword,
      insertText: "return",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: {
        value: "return",
      },
    },
    //testing
    {
      label: "abs",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "abs(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "abs(x)",
    },

    {
      label: "all",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "all(iterable)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "all(iterable)",
    },

    {
      label: "any",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "any(iterable)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "any(iterable)",
    },

    {
      label: "ascii",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "ascii(object)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "ascii(object)",
    },
    {
      label: "bin",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "bin(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "bin(x)",
    },
    {
      label: "bool",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "bool(object)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "bool(object)",
    },
    {
      label: "bytearray",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "bytearray([source[, encoding[, errors]]])",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "bytearray([source[, encoding[, errors]]])",
    },
    {
      label: "bytes",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "bytes(x, encoding, error)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "bytes(x, encoding, error)",
    },
    {
      label: "callable",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "callable(object)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "callable(object)",
    },
    {
      label: "chr",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "chr(i)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "chr(i)",
    },
    {
      label: "classmethod",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "classmethod(function)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "classmethod(function)",
    },
    {
      label: "compile",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "compile(source, filename, mode, flag, dont_inherit, optimize)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "compile(source, filename, mode, flag, dont_inherit, optimize)",
    },
    {
      label: "complex",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "complex(real, imaginary)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "complex(real, imaginary)",
    },
    {
      label: "delattr",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "delattr(object, attribute)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "delattr(object, attribute)",
    },
    {
      label: "dict",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "dict(keyword arguments)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "dict(keyword arguments)",
    },
    {
      label: "dir",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "dir(object)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "dir(object)",
    },
    {
      label: "divmod",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "divmod(divident, divisor)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "divmod(divident, divisor)",
    },
    {
      label: "enumerate",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "enumerate(iterable, start)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "enumerate(iterable, start)",
    },
    {
      label: "eval",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "eval(expression, globals, locals)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "eval(expression, globals, locals)",
    },
    {
      label: "exec",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "exec(object, globals, locals)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "exec(object, globals, locals)",
    },
    {
      label: "filter",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "filter(function, iterable)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "filter(function, iterable)",
    },
    {
      label: "float",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "float(value)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "float(value)",
    },
    {
      label: "format",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "format(value, format)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "format(value, format)",
    },
    {
      label: "frozenset",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "frozenset(iterable)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "frozenset(iterable)",
    },
    {
      label: "getattr",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "getattr(object, attribute, default)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "getattr(object, attribute, default)",
    },
    {
      label: "globals",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "globals()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "globals()",
    },
    {
      label: "hasattr",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "hasattr(object, attribute)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "hasattr(object, attribute)",
    },
    {
      label: "hash",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "hash(object)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "hash(object)",
    },
    {
      label: "help",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "help(object)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "help(object)",
    },
    {
      label: "hex",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "hex(number)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "hex(number)",
    },
    {
      label: "int",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "int(value, base)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "int(value, base)",
    },
    {
      label: "id",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "id(object)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "id(object)",
    },
    {
      label: "input",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "input(prompt)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "input(prompt)",
    },
    {
      label: "isinstance",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "isinstance(object, type)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "isinstance(object, type)",
    },
    {
      label: "issubclass",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "issubclass(object, subclass)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "issubclass(object, subclass)",
    },
    {
      label: "iter",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "iter(object, subclass)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "iter(object, subclass)",
    },
    {
      label: "len",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "len(s)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "len(s)",
    },
    {
      label: "list",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "list([iterable])",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "list([iterable])",
    },
    {
      label: "locals",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "locals()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "locals()",
    },
    {
      label: "map",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "map(function, iterables)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "map(function, iterables)",
    },
    {
      label: "max",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "max(iterable)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "max(iterable)",
    },
    {
      label: "memoryview",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "memoryview(obj)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "memoryview(obj)",
    },
    {
      label: "min",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "min(iterable)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "min(iterable)",
    },
    {
      label: "next",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "next(iterable, default)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "next(iterable, default)",
    },
    {
      label: "object",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "object()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "object()",
    },
    {
      label: "oct",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "oct(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "oct(x)",
    },
    {
      label: "open",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "open(file, mode)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "open(file, mode)",
    },
    {
      label: "ord",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "ord(c)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "ord(c)",
    },
    {
      label: "pow",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "pow(x, y)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "pow(x, y)",
    },
    {
      label: "print",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "print(object(s), separator=separator, end=end, file=file, flush=flush)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "print(object(s), separator=separator, end=end, file=file, flush=flush)",
    },
    {
      label: "property",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "property(fget=None, fset=None, fdel=None, doc=None)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "property(fget=None, fset=None, fdel=None, doc=None)",
    },
    {
      label: "range",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "range(start, stop, step)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "range(start, stop, step)",
    },
    {
      label: "repr",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "repr(object)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "repr(object)",
    },
    {
      label: "reversed",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "reversed(seq)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "reversed(seq)",
    },
    {
      label: "round",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "round(number[, ndigits])",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "round(number[, ndigits])",
    },
    {
      label: "set",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "set(iterable)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "set(iterable)",
    },
    {
      label: "setattr",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "setattr(object, name, value)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "setattr(object, name, value)",
    },
    {
      label: "slice",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "slice(start, end, step)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "slice(start, end, step)",
    },
    {
      label: "sorted",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "sorted(iterable, key=key, reverse=reverse)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "sorted(iterable, key=key, reverse=reverse)",
    },
    {
      label: "staticmethod",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "staticmethod(function)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "staticmethod(function)",
    },
    {
      label: "str",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "str(object, encoding=encoding, errors=errors)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "str(object, encoding=encoding, errors=errors)",
    },
    {
      label: "sum",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "sum(iterable, start)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "sum(iterable, start)",
    },
    {
      label: "super",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "super(type[, object-or-type])",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "super(type[, object-or-type])",
    },
    {
      label: "tuple",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "tuple(iterable)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "tuple(iterable)",
    },
    {
      label: "type",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "type(object, bases, dict)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "type(object, bases, dict)",
    },
    {
      label: "unichr",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "unichr(i)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "unichr(i)",
    },
    {
      label: "vars",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "vars(object)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "vars(object)",
    },
    {
      label: "zip",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "zip(iterator1, iterqator2, iterator3 ...)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "zip(iterator1, iterqator2, iterator3 ...)",
    },
    {
      label: "if",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "if condition:\n  pass",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "if condition:\n  pass",
    },
    {
      label: "ifelif",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "if condition:\n  pass\nelif condition:\n  pass",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "if condition:\n  pass\nelif condition:\n  pass",
    },
    {
      label: "ifelifelse",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "if condition:\n  pass\nelif condition:\n  pass\nelse:\n  pass",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "if condition:\n  pass\nelif condition:\n  pass\nelse:\n  pass",
    },
    {
      label: "ifel",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "if condition:\n  pass\nelse:\n  pass",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "if condition:\n  pass\nelse:\n  pass",
    },
    {
      label: "elif",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "else:\n  pass",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "else:\n  pass",
    },
    {
      label: "ifshort",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "print('A') if a > b else print('A')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "print('A') if a > b else print('A')",
    },
    {
      label: "lambda",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "lambda arguments : expression",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "lambda arguments : expression",
    },
    {
      label: "for",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "for item in range:\n ",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "for item in range:\n ",
    },

    {
      label: "for=>range_function_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "for x in range(6):\n print(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "for x in range(6):\n print(x)",
    },
    {
      label: "for=>for_else",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "for x in range(2, 6):\n print(x)\nelse:\n print('Finally finished!')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "for x in range(2, 6):\n print(x)\nelse:\n print('Finally finished!')",
    },
    {
      label: "while",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "while expression:\n  pass",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "while expression:\n  pass",
    },
    {
      label: "while_else",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "while expression:\n  pass\nelse:\n  pass",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "while expression:\n  pass\nelse:\n  pass",
    },
    {
      label: "while=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "i = 1\nwhile i < 6:\n  print(i)\n  i += 1",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "i = 1\nwhile i < 6:\n  print(i)\n  i += 1",
    },
    {
      label: "while=>break_statement",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "i = 1\nwhile i < 6:\n print(i)\n  if i == 3:\n    break\n  i += 1",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "i = 1\nwhile i < 6:\n print(i)\n  if i == 3:\n    break\n  i += 1",
    },
    {
      label: "while=>continue_statement",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "i = 1\nwhile i < 6:\n  i += 1\n  print(i)\n  if i == 3:\n    continue\n  print(i)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "i = 1\nwhile i < 6:\n  i += 1\n  print(i)\n  if i == 3:\n    continue\n  print(i)",
    },
    {
      label: "function",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "def name(args):\n pass",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "def name(args):\n pass",
    },
    {
      label: "def",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "def name(args):\n pass",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "def name(args):\n pass",
    },
    {
      label: "capitalize",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".capitalize()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".capitalize()",
    },
    {
      label: "casefold",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".casefold()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".casefold()",
    },
    {
      label: "center",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".center()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".center()",
    },
    {
      label: "string.count",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".count()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".count()",
    },
    {
      label: "encode",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".encode()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".encode()",
    },
    {
      label: "endswith",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".endswith()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".endswith()",
    },
    {
      label: "expandtabs",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".expandtabs()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".expandtabs()",
    },
    {
      label: "find",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".find()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".find()",
    },
    {
      label: "format",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".format()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".format()",
    },
    {
      label: "format_map",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".format_map()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".format_map()",
    },
    {
      label: "index",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".index()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".index()",
    },
    {
      label: "isalnum",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".isalnum()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".isalnum()",
    },
    {
      label: "isalpha",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".isalpha()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".isalpha()",
    },
    {
      label: "isdecimal",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".isdecimal()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".isdecimal()",
    },
    {
      label: "isdigit",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".isdigit()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".isdigit()",
    },
    {
      label: "isidentifier",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".isidentifier()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".isidentifier()",
    },
    {
      label: "islower",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".islower()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".islower()",
    },
    {
      label: "isnumeric",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".isnumeric()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".isnumeric()",
    },
    {
      label: "isprintable",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".isprintable()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".isprintable()",
    },
    {
      label: "isspace",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".isspace()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".isspace()",
    },
    {
      label: "istitle",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".istitle()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".istitle()",
    },
    {
      label: "isupper",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".isupper()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".isupper()",
    },
    {
      label: "join",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".join()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".join()",
    },
    {
      label: "ljust",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".ljust()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".ljust()",
    },
    {
      label: "lower",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".lower()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".lower()",
    },
    {
      label: "lstrip",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".lstrip()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".lstrip()",
    },
    {
      label: "maketrans",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".maketrans()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".maketrans()",
    },
    {
      label: "partition",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".partition()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".partition()",
    },
    {
      label: "replace",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".replace(x, y)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".replace(x, y)",
    },
    {
      label: "rfind",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".rfind()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".rfind()",
    },
    {
      label: "rindex",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".rindex()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".rindex()",
    },
    {
      label: "rpartition",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".rpartition()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".rpartition()",
    },
    {
      label: "rsplit",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".rsplit()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".rsplit()",
    },
    {
      label: "split",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".split()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".split()",
    },
    {
      label: "splitlines",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".splitlines()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".splitlines()",
    },
    {
      label: "string.splitlines=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Thank you for the music\nWelcome to the jungle'\nx = txt.splitlines()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Thank you for the music\nWelcome to the jungle'\nx = txt.splitlines()\nprint(x)",
    },
    {
      label: "string.splitlines=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Thank you for the music\nWelcome to the jungle'\nx = txt.splitlines(True)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Thank you for the music\nWelcome to the jungle'\nx = txt.splitlines(True)\nprint(x)",
    },
    {
      label: "startswith",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".startswith()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".startswith()",
    },
    {
      label: "swapcase",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".swapcase()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".swapcase()",
    },
    {
      label: "title",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".title()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".title()",
    },
    {
      label: "translate",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".translate()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".translate()",
    },
    {
      label: "upper",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".upper()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".upper()",
    },
    {
      label: "string.upper=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'Hello my friends'\nx = txt.upper()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = 'Hello my friends'\nx = txt.upper()\nprint(x)",
    },
    {
      label: "zfill",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".zfill()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".zfill()",
    },
    {
      label: "append",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".append()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".append()",
    },
    {
      label: "clear",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".clear()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".clear()",
    },
    {
      label: "copy",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".copy()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".copy()",
    },
    {
      label: "list.count",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".count",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".count",
    },
    {
      label: "extend",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".extend()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".extend()",
    },
    {
      label: "index",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".index()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".index()",
    },
    {
      label: "insert",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".insert()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".insert()",
    },
    {
      label: "pop",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".pop()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".pop()",
    },
    {
      label: "remove",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".remove()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".remove()",
    },
    {
      label: "reverse",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".reverse()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".reverse()",
    },
    {
      label: "sort",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".sort()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".sort()",
    },
    {
      label: "comprehensions",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "[ expression for item in list if conditional ]",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "[ expression for item in list if conditional ]",
    },
    {
      label: "clear",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".clear()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".clear()",
    },
    {
      label: "copy",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".copy()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".copy()",
    },
    {
      label: "fromkeys",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".fromkeys(x, y)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".fromkeys(x, y)",
    },

    {
      label: "get",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".get()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".get()",
    },
    {
      label: "items",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".items()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".items()",
    },
    {
      label: "keys",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".keys()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".keys()",
    },
    {
      label: "pop",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".pop()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".pop()",
    },
    {
      label: "popitem",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".popitem()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".popitem()",
    },
    {
      label: "setdefault",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".setdefault()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".setdefault()",
    },
    {
      label: "update",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".update()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".update()",
    },
    {
      label: "values",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".values()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".values()",
    },
    {
      label: "tuple.count",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".count(value)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".count(value)",
    },
    {
      label: "index",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".index(value)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".index(value)",
    },
    {
      label: "add",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".add()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".add()",
    },
    {
      label: "clear",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".clear()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".clear()",
    },
    {
      label: "copy",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".copy()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".copy()",
    },
    {
      label: "difference",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x.difference(y)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x.difference(y)",
    },
    {
      label: "difference_update",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x.difference_update(y)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x.difference_update(y)",
    },
    {
      label: "discard",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".discard()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".discard()",
    },
    {
      label: "intersection",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x.intersection(y)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x.intersection(y)",
    },
    {
      label: "intersection_update",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x.intersection_update(y)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x.intersection_update(y)",
    },
    {
      label: "isdisjoint",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x.isdisjoint(y)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x.isdisjoint(y)",
    },
    {
      label: "issubset",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x.issubset(y)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x.issubset(y)",
    },
    {
      label: "issuperset",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x.issuperset(y)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x.issuperset(y)",
    },
    {
      label: "pop",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".pop()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".pop()",
    },
    {
      label: "remove",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: ".remove()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: ".remove()",
    },
    {
      label: "symmetric_difference",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x.symmetric_difference(y)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x.symmetric_difference(y)",
    },
    {
      label: "symmetric_difference_update",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x.symmetric_difference_update(y)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x.symmetric_difference_update(y)",
    },
    {
      label: "union",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x.union(y)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x.union(y)",
    },
    {
      label: "update",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x.update(y)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x.update(y)",
    },
    {
      label: "class",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "class MyClass:\n  pass",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "class MyClass:\n  pass",
    },
    {
      label: "self",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "self",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "self",
    },
    {
      label: "__init__",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "def __init__(self, name, age):\n  self.name = name\n  self.age = age",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "def __init__(self, name, age):\n  self.name = name\n  self.age = age",
    },
    {
      label: "__iter__",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "def __iter__(self):\n  self.a = 1\n  return self",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "def __iter__(self):\n  self.a = 1\n  return self",
    },
    {
      label: "__next__",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "def __next__(self):\n  x = self.a\n  self.a += 1\n  return x",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "def __next__(self):\n  x = self.a\n  self.a += 1\n  return x",
    },
    {
      label: "import",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "import mymodule as mx",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "import mymodule as mx",
    },
    {
      label: "tryexcept",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "try:\n  print(x)\nexcept:\n  print('An exception occurred')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "try:\n  print(x)\nexcept:\n  print('An exception occurred')",
    },
    {
      label: "tryexceptfinally",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "try:\n  print(x)\nexcept:\n  print('Something went wrong')\nfinally:\n  print('The try except is finished')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "try:\n  print(x)\nexcept:\n  print('Something went wrong')\nfinally:\n  print('The try except is finished')",
    },
    {
      label: "openFile",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "f = open('demofile.txt', 'r')\nprint(f.read())",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "f = open('demofile.txt', 'r')\nprint(f.read())",
    },
    {
      label: "openFileReadLine",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "f = open('demofile.txt', 'r')\nprint(f.readline())",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "f = open('demofile.txt', 'r')\nprint(f.readline())",
    },
    {
      label: "writeExistFile",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "f = open('demofile.txt', 'a')\nf.write('Now the file has one more line!')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "f = open('demofile.txt', 'a')\nf.write('Now the file has one more line!')",
    },
    {
      label: "writeOwerWrite",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "f = open('demofile.txt', 'w')\nf.write('Woops! I have deleted the content!')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "f = open('demofile.txt', 'w')\nf.write('Woops! I have deleted the content!')",
    },
    {
      label: "createFileIfDoesNotExist",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "f = open('myfile.txt', 'w')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "f = open('myfile.txt', 'w')",
    },
    {
      label: "createFile",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "f = open('myfile.txt', 'x')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "f = open('myfile.txt', 'x')",
    },
    {
      label: "deleteFile",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "#import os\nos.remove('demofile.txt')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "#import os\nos.remove('demofile.txt')",
    },
    // startwith special character
    {
      label: "name",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "__name__",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "__name__: str",
    },
  ];
};
