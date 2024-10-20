export const resizeList = ["min", "max"];
export const editorList = ["ace", "monaco"];
export const monacoEditorThemeList = ["vs", "vs-dark", "hc-black"];
export const defaultEditorSourceCode = `import GUI
import HAL
# Enter sequential code!

while True:
    # Enter iterative code!
    

`;

// Know about pylint Errors with type
// https://gist.github.com/codezerro/f7f696702ee7ea12782d9af3f9bb4f4c
// pylint disable error
export const pylint_error = ["E0401", "E1101"];
export const pylint_warning = ["W0611"];
export const pylint_convention = ["C0114", "C0303", "C0304", "C0305", "C0411"];
export const pylint_refactor = [];
export const pylint_fatal = [];

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

/*
{
        type: "method",
        label: "",
        code:"",
        descriptions:
          "",
      },
*/

/*
_montecarlo_laser_loc: {
  hal: [],
  gui: [],
}

*/

export const getAllSnippets = ({ monaco, range }) => {
  return [
    // RA
    {
      label: "HAL",
      kind: monaco.languages.CompletionItemKind.Class,
      insertText: "HAL",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "HAL",
      sortText: "0001", // High priority
      preselect: true, // Preselected suggestion
    },
    {
      label: "GUI",
      kind: monaco.languages.CompletionItemKind.Class,
      insertText: "GUI",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "GUI",
      sortText: "0002", // High priority
      preselect: true, // Preselected suggestion
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
      label: "built_in.abs=>int",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = abs(-7.25)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = abs(-7.25)\nprint(x)",
    },
    {
      label: "built_in.abs=>float",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = abs(-20)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = abs(-20)\nprint(x)",
    },
    {
      label: "built_in.abs=>complex",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = abs((3 - 4j))\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = abs((3 - 4j))\nprint(x)",
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
      label: "built_in.all=>list_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "mylist = [True, True, True]\nx = all(mylist)\nprint(x)\n\n# Return True",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "mylist = [True, True, True]\nx = all(mylist)\nprint(x)\n\n# Return True",
    },
    {
      label: "built_in.all=>list_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "mylist = [0, 1, 1]\nx = all(mylist)\nprint(x)\n\n# Returns False because 0 is the same as False",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "mylist = [0, 1, 1]\nx = all(mylist)\nprint(x)\n\n# Returns False because 0 is the same as False",
    },
    {
      label: "built_in.all=>tuple",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "mytuple = (0, True, False)\nx = all(mytuple)\nprint(x)\n\n# Returns False because both the first and the third items are False",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "mytuple = (0, True, False)\nx = all(mytuple)\nprint(x)\n\n# Returns False because both the first and the third items are False",
    },
    {
      label: "built_in.all=>set",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "myset = {0, 1, 0}\nx = all(myset)\nprint(x)\n\n# Returns False because both the first and the third items are False",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "myset = {0, 1, 0}\nx = all(myset)\nprint(x)\n\n# Returns False because both the first and the third items are False",
    },
    {
      label: "built_in.all=>dictionary",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "mydict = {0 : 'Apple', 1 : 'Orange'}\nx = all(mydict)\nprint(x)\n\n# Returns False because the first key is false.\n# For dictionaries the all() function checks the keys, not the values.",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "mydict = {0 : 'Apple', 1 : 'Orange'}\nx = all(mydict)\nprint(x)\n\n# Returns False because the first key is false.\n# For dictionaries the all() function checks the keys, not the values.",
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
      label: "built_in.any=>list_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "mylist = [False, True, False]\nx = any(mylist)\nprint(x)\n\n# Return True",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "mylist = [False, True, False]\nx = any(mylist)\nprint(x)\n\n# Return True",
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
      label: "built_in.ascii=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = ascii('My name is Ståle')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = ascii('My name is Ståle')\nprint(x)",
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
      label: "built_in.bin=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = bin(36)\nprint(x)\n# Result : 0b100100",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = bin(36)\nprint(x)\n# Result : 0b100100",
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
      label: "built_in.bool=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = bool(1)\nprint(x)\n# Result : True",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = bool(1)\nprint(x)\n# Result : True",
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
      label: "built_in.bytearray=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = bytearray(4)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = bytearray(4)\nprint(x)",
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
      label: "built_in.bytes=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = x = bytes(4)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = x = bytes(4)\nprint(x)",
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
      label: "built_in.callable=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "def x():\na = 5\n\nprint(callable(x))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "def x():\na = 5\n\nprint(callable(x))",
    },
    {
      label: "built_in.callable=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = 5\n\nprint(callable(x))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = 5\n\nprint(callable(x))",
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
      label: "built_in.chr=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = chr(97)\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = chr(97)\n\nprint(x)",
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
      label: "built_in.compile=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "mytext = 'print(55)'\nx = compile('mytext', 'test', 'eval')\nexec(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "mytext = 'print(55)'\nx = compile('mytext', 'test', 'eval')\nexec(x)",
    },
    {
      label: "built_in.compile=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "mytext = 'print(55)\nprint(88)'\nx = compile('mytext', 'test', 'exec')\nexec(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "mytext = 'print(55)\nprint(88)'\nx = compile('mytext', 'test', 'exec')\nexec(x)",
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
      label: "built_in.complex=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = complex(3, 5)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = complex(3, 5)\nprint(x)",
    },
    {
      label: "built_in.complex=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = complex('3+5j')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = complex('3+5j')\nprint(x)",
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
      label: "built_in.delattr=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Person:\n name = 'John'\n age = 36\n country = 'Norway'\n\ndelattr(Person, 'age')\n# The Person object will no longer contain an age property",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Person:\n name = 'John'\n age = 36\n country = 'Norway'\n\ndelattr(Person, 'age')\n# The Person object will no longer contain an age property",
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
      label: "built_in.dict=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = dict(name = 'John', age = 36, country = 'Norway')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = dict(name = 'John', age = 36, country = 'Norway')\nprint(x)",
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
      label: "built_in.dir=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Person:\n name = 'John'\n age = 36\n country = 'Norway'\nprint(dir(Person))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Person:\n name = 'John'\n age = 36\n country = 'Norway'\nprint(dir(Person))",
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
      label: "built_in.divmod=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = divmod(5, 2)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = divmod(5, 2)\nprint(x)",
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
      label: "built_in.enumerate=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = ('apple', 'banana', 'cherry')\ny = enumerate(x)\n\nprint(list(y))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = ('apple', 'banana', 'cherry')\ny = enumerate(x)\n\nprint(list(y))",
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
      label: "built_in.eval=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = 'print(55)'\neval(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = 'print(55)'\neval(x)",
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
      label: "built_in.exec=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = 'age = 25\nprint(age)'\nexec(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = 'age = 25\nprint(age)'\nexec(x)",
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
      label: "built_in.filter=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "ages = [5, 12, 17, 18, 24, 32]\n\ndef myFunc(x):\n if x < 18:\n   return False\n else:\n   return True\n\nadults = filter(myFunc, ages)\n\nfor x in adults:\n print(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "ages = [5, 12, 17, 18, 24, 32]\n\ndef myFunc(x):\n if x < 18:\n   return False\n else:\n   return True\n\nadults = filter(myFunc, ages)\n\nfor x in adults:\n print(x)",
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
      label: "built_in.float=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = float(3)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = float(3)\nprint(x)",
    },
    {
      label: "built_in.float=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = float('3.500')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = float('3.500')\nprint(x)",
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
      label: "built_in.format=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = format(0.5, '%')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = format(0.5, '%')\nprint(x)",
    },
    {
      label: "built_in.format=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = format(255, 'x')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = format(255, 'x')\nprint(x)",
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
      label: "built_in.frozenset=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "mylist = ['apple', 'banana', 'cherry']\nx = frozenset(mylist)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "mylist = ['apple', 'banana', 'cherry']\nx = frozenset(mylist)\nprint(x)",
    },
    {
      label: "built_in.frozenset=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "mylist = ['apple', 'banana', 'cherry']\nx = frozenset(mylist)\nx[1] = 'strawberry'\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "mylist = ['apple', 'banana', 'cherry']\nx = frozenset(mylist)\nx[1] = 'strawberry'\nprint(x)",
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
      label: "built_in.frozenset=>_3",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Person:\n name = 'John'\n age = 36\n country = 'Norway'\n\nx = getattr(Person, 'age')\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Person:\n name = 'John'\n age = 36\n country = 'Norway'\n\nx = getattr(Person, 'age')\n\nprint(x)",
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
      label: "built_in.globals=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = globals()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = globals()\nprint(x)",
    },
    {
      label: "built_in.globals=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = globals()\nprint(x['__file__'])",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = globals()\nprint(x['__file__'])",
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
      label: "built_in.hasattr=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Person:\n name = 'John'\n age = 36\n country = 'Norway'\n\nx = hasattr(Person, 'age')\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Person:\n name = 'John'\n age = 36\n country = 'Norway'\n\nx = hasattr(Person, 'age')\n\nprint(x)",
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
      label: "built_in.hex=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = hex(255)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = hex(255)\nprint(x)",
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
      label: "built_in.int=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = int(3.5)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = int(3.5)\nprint(x)",
    },
    {
      label: "built_in.int=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = int('12')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = int('12')\nprint(x)",
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
      label: "built_in.id=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Foo:\nb = 5\n\ndummyFoo = Foo()\nprint('id of dummyFoo =',id(dummyFoo))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Foo:\nb = 5\n\ndummyFoo = Foo()\nprint('id of dummyFoo =',id(dummyFoo))",
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
      label: "built_in.input=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = input('Enter your name:')\nprint('Hello, ' + x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = input('Enter your name:')\nprint('Hello, ' + x)",
    },
    {
      label: "built_in.input=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "print('Enter your name:')\nx = input()\nprint('Hello, ' + x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "print('Enter your name:')\nx = input()\nprint('Hello, ' + x)",
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
      label: "built_in.isinstance=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = isinstance(5, int)\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = isinstance(5, int)\n\nprint(x)",
    },
    {
      label: "built_in.isinstance=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = isinstance('Hello', (float, int, str, list, dict, tuple))\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = isinstance('Hello', (float, int, str, list, dict, tuple))\n\nprint(x)",
    },
    {
      label: "built_in.isinstance=>_3",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class myObj:\n name = 'John'\n\ny = myObj()\n\nx = isinstance(y, myObj)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class myObj:\n name = 'John'\n\ny = myObj()\n\nx = isinstance(y, myObj)\nprint(x)",
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
      label: "built_in.issubclass=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class myAge:\n age = 36\n\nclass myObj(myAge):\n name = 'John'\n age = myAge\n\n x = issubclass(myObj, myAge)\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class myAge:\n age = 36\n\nclass myObj(myAge):\n name = 'John'\n age = myAge\n\n x = issubclass(myObj, myAge)\n\nprint(x)",
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
      label: "built_in.iter=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = iter(['apple', 'banana', 'cherry'])\nprint(next(x))\nprint(next(x))\nprint(next(x))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = iter(['apple', 'banana', 'cherry'])\nprint(next(x))\nprint(next(x))\nprint(next(x))",
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
      label: "built_in.len=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "mylist = ['apple', 'banana', 'cherry']\nx = len(mylist)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "mylist = ['apple', 'banana', 'cherry']\nx = len(mylist)",
    },
    {
      label: "built_in.len=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "mylist = 'Hello'\nx = len(mylist)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "mylist = 'Hello'\nx = len(mylist)",
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
      label: "built_in.list=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = list(('apple', 'banana', 'cherry'))\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = list(('apple', 'banana', 'cherry'))\nprint(x)",
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
      label: "built_in.locals=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = locals()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = locals()\nprint(x)",
    },
    {
      label: "built_in.locals=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = locals()\nprint(x['__file__'])",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = locals()\nprint(x['__file__'])",
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
      label: "built_in.map=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "def myfunc(n):\n return len(n)\n\nx = map(myfunc, ('apple', 'banana', 'cherry'))\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "def myfunc(n):\n return len(n)\n\nx = map(myfunc, ('apple', 'banana', 'cherry'))\n\nprint(x)",
    },
    {
      label: "built_in.map=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "def myfunc(a, b):\n return a + b\n\nx = map(myfunc, ('apple', 'banana', 'cherry'), ('orange', 'lemon', 'pineapple'))\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "def myfunc(a, b):\n return a + b\n\nx = map(myfunc, ('apple', 'banana', 'cherry'), ('orange', 'lemon', 'pineapple'))\n\nprint(x)",
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
      label: "built_in.max=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = max(5, 10)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = max(5, 10)\nprint(x)",
    },
    {
      label: "built_in.max=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = max('Mike', 'John', 'Vicky')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = max('Mike', 'John', 'Vicky')\nprint(x)",
    },
    {
      label: "built_in.max=>_3",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "a = (1, 5, 3, 9)\nx = max(a)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "a = (1, 5, 3, 9)\nx = max(a)\nprint(x)",
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
      label: "built_in.memoryview=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = memoryview(b'Hello')\nprint(x)\n\n#return the Unicode of the first character\nprint(x[0])\n\n#return the Unicode of the second character\nprint(x[1])",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = memoryview(b'Hello')\nprint(x)\n\n#return the Unicode of the first character\nprint(x[0])\n\n#return the Unicode of the second character\nprint(x[1])",
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
      label: "built_in.min=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = min(5, 10)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = min(5, 10)\nprint(x)",
    },
    {
      label: "built_in.min=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = min('Mike', 'John', 'Vicky')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = min('Mike', 'John', 'Vicky')\nprint(x)",
    },
    {
      label: "built_in.min=>_3",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "a = (1, 5, 3, 9)\nx = min(a)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "a = (1, 5, 3, 9)\nx = min(a)\nprint(x)",
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
      label: "built_in.next=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "mylist = iter(['apple', 'banana', 'cherry'])\nx = next(mylist)\nprint(x)\nx = next(mylist)\nprint(x)\nx = next(mylist)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "mylist = iter(['apple', 'banana', 'cherry'])\nx = next(mylist)\nprint(x)\nx = next(mylist)\nprint(x)\nx = next(mylist)\nprint(x)",
    },
    {
      label: "built_in.next=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "mylist = iter(['apple', 'banana', 'cherry'])\nx = next(mylist, 'orange')\nprint(x)\nx = next(mylist, 'orange')\nprint(x)\nx = next(mylist, 'orange')\nprint(x)\nx = next(mylist, 'orange')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "mylist = iter(['apple', 'banana', 'cherry'])\nx = next(mylist, 'orange')\nprint(x)\nx = next(mylist, 'orange')\nprint(x)\nx = next(mylist, 'orange')\nprint(x)\nx = next(mylist, 'orange')\nprint(x)",
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
      label: "built_in.object=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = object()\nprint(dir(x))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = object()\nprint(dir(x))",
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
      label: "built_in.oct=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = oct(12)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = oct(12)\nprint(x)",
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
      label: "built_in.open=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "f = open('demofile.txt', 'r')\nprint(f.read())",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "f = open('demofile.txt', 'r')\nprint(f.read())",
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
      label: "built_in.ord=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x=ord('a')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x=ord('a')\nprint(x)",
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
      label: "built_in.pow=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x=pow(2,5)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x=pow(2,5)\nprint(x)",
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
      label: "built_in.print=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "print('Hello', 'how are you?')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "print('Hello', 'how are you?')",
    },
    {
      label: "built_in.print=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = ('apple', 'banana', 'cherry')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = ('apple', 'banana', 'cherry')\nprint(x)",
    },
    {
      label: "built_in.print=>_3",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "print('Hello', 'how are you?', sep=' ---')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "print('Hello', 'how are you?', sep=' ---')",
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
      label: "built_in.property=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class C:\n    def __init__(self):\n        self._x = None\n    def getx(self):\n        return self._x\n    def setx(self, value):\n        self._x = value\n    def delx(self):\n        del self._x\n    x = property(getx, setx, delx, 'I'm the 'x' property.')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class C:\n    def __init__(self):\n        self._x = None\n    def getx(self):\n        return self._x\n    def setx(self, value):\n        self._x = value\n    def delx(self):\n        del self._x\n    x = property(getx, setx, delx, 'I'm the 'x' property.')",
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
      label: "built_in.range=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = range(6)\nfor n in x:\n  print(n)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = range(6)\nfor n in x:\n  print(n)",
    },
    {
      label: "built_in.range=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = range(3, 6)\nfor n in x:\n  print(n)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = range(3, 6)\nfor n in x:\n  print(n)",
    },
    {
      label: "built_in.range=>_3",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = range(3, 20, 2)\nfor n in x:\n  print(n)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = range(3, 20, 2)\nfor n in x:\n  print(n)",
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
      label: "built_in.reversed=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "alph = ['a', 'b', 'c', 'd']\nralph = reversed(alph)\nfor x in ralph:\n  print(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "alph = ['a', 'b', 'c', 'd']\nralph = reversed(alph)\nfor x in ralph:\n  print(x)",
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
      label: "built_in.round=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = round(5.76543, 2)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = round(5.76543, 2)\nprint(x)",
    },
    {
      label: "built_in.round=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = round(5.76543)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = round(5.76543)\nprint(x)",
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
      label: "built_in.set=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = set(('apple', 'banana', 'cherry'))\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = set(('apple', 'banana', 'cherry'))\nprint(x)",
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
      label: "built_in.setattr=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Person:\n  name = 'John'\n  age = 36\n  country = 'Norway'\nsetattr(Person, 'age', 40)\n# The age property will now have the value: 40\nx = getattr(Person, 'age')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Person:\n  name = 'John'\n  age = 36\n  country = 'Norway'\nsetattr(Person, 'age', 40)\n# The age property will now have the value: 40\nx = getattr(Person, 'age')\nprint(x)",
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
      label: "built_in.slice=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = ('a', 'b', 'c', 'd', 'e', 'f', 'g', 'h')\nx = slice(2)\nprint(a[x])",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = ('a', 'b', 'c', 'd', 'e', 'f', 'g', 'h')\nx = slice(2)\nprint(a[x])",
    },
    {
      label: "built_in.slice=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = ('a', 'b', 'c', 'd', 'e', 'f', 'g', 'h')\nx = slice(3, 5)\nprint(a[x])",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = ('a', 'b', 'c', 'd', 'e', 'f', 'g', 'h')\nx = slice(3, 5)\nprint(a[x])",
    },
    {
      label: "built_in.slice=>_3",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = ('a', 'b', 'c', 'd', 'e', 'f', 'g', 'h')\nx = slice(0, 8, 3)\nprint(a[x])",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = ('a', 'b', 'c', 'd', 'e', 'f', 'g', 'h')\nx = slice(0, 8, 3)\nprint(a[x])",
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
      label: "built_in.sorted=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = ('b', 'g', 'a', 'd', 'f', 'c', 'h', 'e')\nx = sorted(a)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = ('b', 'g', 'a', 'd', 'f', 'c', 'h', 'e')\nx = sorted(a)\nprint(x)",
    },
    {
      label: "built_in.sorted=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = ('a', 'b', 'c', 'd', 'e', 'f', 'g', 'h')\nx = sorted(a, reverse=True)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = ('a', 'b', 'c', 'd', 'e', 'f', 'g', 'h')\nx = sorted(a, reverse=True)\nprint(x)",
    },
    {
      label: "built_in.sorted=>_3",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = ('b', 'g', 'a', 'd', 'f', 'c', 'h', 'e')\nx = sorted(a)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = ('b', 'g', 'a', 'd', 'f', 'c', 'h', 'e')\nx = sorted(a)\nprint(x)",
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
      label: "built_in.str=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = str(3.5)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = str(3.5)\nprint(x)",
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
      label: "built_in.sum=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "a = (1, 2, 3, 4, 5)\nx = sum(a)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "a = (1, 2, 3, 4, 5)\nx = sum(a)\nprint(x)",
    },
    {
      label: "built_in.sum=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "a = (1, 2, 3, 4, 5)\nx = sum(a, 7)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "a = (1, 2, 3, 4, 5)\nx = sum(a, 7)\nprint(x)",
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
      label: "built_in.tuple=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = tuple(('apple', 'banana', 'cherry'))\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = tuple(('apple', 'banana', 'cherry'))\nprint(x)",
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
      label: "built_in.type=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = ('apple', 'banana', 'cherry')\nb = 'Hello World'\nc = 33\nx = type(a)\ny = type(b)\nz = type(c)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = ('apple', 'banana', 'cherry')\nb = 'Hello World'\nc = 33\nx = type(a)\ny = type(b)\nz = type(c)",
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
      label: "built_in.vars=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Person:\n  name = 'John'\n  age = 36\n  country = 'norway'\nx = vars(Person)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Person:\n  name = 'John'\n  age = 36\n  country = 'norway'\nx = vars(Person)",
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
      label: "built_in.zip=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = ('John', 'Charles', 'Mike')\nb = ('Jenny', 'Christy', 'Monica')\nx = zip(a, b)\n#use the tuple() function to display a readable version of the result:\nprint(tuple(x))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = ('John', 'Charles', 'Mike')\nb = ('Jenny', 'Christy', 'Monica')\nx = zip(a, b)\n#use the tuple() function to display a readable version of the result:\nprint(tuple(x))",
    },
    {
      label: "built_in.zip=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = ('John', 'Charles', 'Mike')\nb = ('Jenny', 'Christy', 'Monica', 'Vicky')\nx = zip(a, b)\n#use the tuple() function to display a readable version of the result:\nprint(tuple(x))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = ('John', 'Charles', 'Mike')\nb = ('Jenny', 'Christy', 'Monica', 'Vicky')\nx = zip(a, b)\n#use the tuple() function to display a readable version of the result:\nprint(tuple(x))",
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
      label: "for=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = ['apple', 'banana', 'cherry']\nfor x in fruits:\n  print(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = ['apple', 'banana', 'cherry']\nfor x in fruits:\n  print(x)",
    },
    {
      label: "for=>through_a_string",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "for x in 'banana':\n  print(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "for x in 'banana':\n  print(x)",
    },
    {
      label: "for=>break_statement",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = ['apple', 'banana', 'cherry']\nfor x in fruits:\n print(x)\n if x == 'banana':\n   break",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = ['apple', 'banana', 'cherry']\nfor x in fruits:\n print(x)\n if x == 'banana':\n   break",
    },
    {
      label: "for=>continue_statement",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = ['apple', 'banana', 'cherry']\nfor x in fruits:\n print(x)\n if x == 'banana':\n   continue\n print(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = ['apple', 'banana', 'cherry']\nfor x in fruits:\n print(x)\n if x == 'banana':\n   continue\n print(x)",
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
      label: "for=>range_function_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "for x in range(2, 6):\n print(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "for x in range(2, 6):\n print(x)",
    },
    {
      label: "for=>range_function_3",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "for x in range(2, 30, 3):\n print(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "for x in range(2, 30, 3):\n print(x)",
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
      label: "for=>nested_loops",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "adj = ['red', 'big', 'tasty']\nfruits = ['apple', 'banana', 'cherry']\nfor x in adj:\n  for y in fruits:\n    print(x, y)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "adj = ['red', 'big', 'tasty']\nfruits = ['apple', 'banana', 'cherry']\nfor x in adj:\n  for y in fruits:\n    print(x, y)",
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
      label: "def=>with_default_value",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "def name(name, lastName='john')\n pass",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "def name(name, lastName='john')\n pass",
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
      label: "string.capitalize=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = hello, and welcome to my world.\n\nx = txt.capitalize()\n\nprint (x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = hello, and welcome to my world.\n\nx = txt.capitalize()\n\nprint (x)",
    },
    {
      label: "string.capitalize=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = '36 is my age.'\n\nx = txt.capitalize()\n\nprint (x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = '36 is my age.'\n\nx = txt.capitalize()\n\nprint (x)",
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
      label: "string.casefold=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, And Welcome To My World!'\n\nx = txt.casefold()\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, And Welcome To My World!'\n\nx = txt.casefold()\n\nprint(x)",
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
      label: "string.center=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'banana'\n\nx = txt.center(20)\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = 'banana'\n\nx = txt.center(20)\n\nprint(x)",
    },
    {
      label: "string.center=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'banana'\n\nx = txt.center(20,'O')\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = 'banana'\n\nx = txt.center(20,'O')\n\nprint(x)",
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
      label: "string.count=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'I love apples, apple are my favorite fruit'\n\nx = txt.count('apple')\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'I love apples, apple are my favorite fruit'\n\nx = txt.count('apple')\n\nprint(x)",
    },
    {
      label: "string.count=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'I love apples, apple are my favorite fruit'\n\nx = txt.count('apple', 10, 24)\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'I love apples, apple are my favorite fruit'\n\nx = txt.count('apple', 10, 24)\n\nprint(x)",
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
      label: "string.encode=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'My name is Ståle'\n\nx = txt.encode()\n\nprint()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = 'My name is Ståle'\n\nx = txt.encode()\n\nprint()",
    },
    {
      label: "string.encode=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'My name is Ståle'\n\nprint(txt.encode(encoding='ascii',errors='backslashreplace')\nprint(txt.encode(encoding='ascii',errors='ignore')\nprint(txt.encode(encoding='ascii',errors='namereplace')\nprint(txt.encode(encoding='ascii',errors='replace')\nprint(txt.encode(encoding='ascii',errors='xmlcharrefreplace')\nprint(txt.encode(encoding='ascii',errors='strict')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'My name is Ståle'\n\nprint(txt.encode(encoding='ascii',errors='backslashreplace')\nprint(txt.encode(encoding='ascii',errors='ignore')\nprint(txt.encode(encoding='ascii',errors='namereplace')\nprint(txt.encode(encoding='ascii',errors='replace')\nprint(txt.encode(encoding='ascii',errors='xmlcharrefreplace')\nprint(txt.encode(encoding='ascii',errors='strict')",
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
      label: "string.endswith=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, And Welcome To My World!'\n\nx = txt.endswith('.')\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, And Welcome To My World!'\n\nx = txt.endswith('.')\n\nprint(x)",
    },
    {
      label: "string.endswith=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, And Welcome To My World!'\n\nx = txt.endswith('my world.', 5, 11)\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, And Welcome To My World!'\n\nx = txt.endswith('my world.', 5, 11)\n\nprint(x)",
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
      label: "string.expandtabs=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'H\te\tl\tl\to'\n\nx = txt.expandtabs(2)\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'H\te\tl\tl\to'\n\nx = txt.expandtabs(2)\n\nprint(x)",
    },
    {
      label: "string.expandtabs=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'H\te\tl\tl\to'\n\nprint(txt)\nprint(txt.expandtabs())\nprint(txt.expandtabs(2))\nprint(txt.expandtabs(4))\nprint(txt.expandtabs(10))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'H\te\tl\tl\to'\n\nprint(txt)\nprint(txt.expandtabs())\nprint(txt.expandtabs(2))\nprint(txt.expandtabs(4))\nprint(txt.expandtabs(10))",
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
      label: "string.find=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, welcome to my world.'\n\nx = txt.find('welcome')\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, welcome to my world.'\n\nx = txt.find('welcome')\n\nprint(x)",
    },
    {
      label: "string.find=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, And Welcome To My World!'\n\nx = txt.find('e')\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, And Welcome To My World!'\n\nx = txt.find('e')\n\nprint(x)",
    },
    {
      label: "string.find=>_3",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, And Welcome To My World!'\n\nx = txt.find('e', 5, 10)\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, And Welcome To My World!'\n\nx = txt.find('e', 5, 10)\n\nprint(x)",
    },
    {
      label: "string.find=>_4",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, And Welcome To My World!'\n\nprint(txt.find('q'))\nprint(txt.index('q'))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, And Welcome To My World!'\n\nprint(txt.find('q'))\nprint(txt.index('q'))",
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
      label: "string.format=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "# default arguments\nprint('Hello {}, your balance is {}.'.format('Adam', 230.2346))\n\n# positional arguments\nprint('Hello {0}, your balance is {1}.'.format('Adam', 230.2346))\n\n# keyword arguments\nprint('Hello {name}, your balance is {blc}.'.format(name='Adam', blc=230.2346))\n\n# mixed arguments\nprint('Hello {0}, your balance is {blc}.'.format('Adam', blc=230.2346))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "# default arguments\nprint('Hello {}, your balance is {}.'.format('Adam', 230.2346))\n\n# positional arguments\nprint('Hello {0}, your balance is {1}.'.format('Adam', 230.2346))\n\n# keyword arguments\nprint('Hello {name}, your balance is {blc}.'.format(name='Adam', blc=230.2346))\n\n# mixed arguments\nprint('Hello {0}, your balance is {blc}.'.format('Adam', blc=230.2346))",
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
      label: "string.format_map=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "point = {'x':4,'y':-5}\nprint('{x} {y}'.format_map(point))\n\npoint = {'x':4,'y':-5, 'z': 0}\nprint('{x} {y} {z}'.format_map(point))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "point = {'x':4,'y':-5}\nprint('{x} {y}'.format_map(point))\n\npoint = {'x':4,'y':-5, 'z': 0}\nprint('{x} {y} {z}'.format_map(point))",
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
      label: "string.index=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, welcome to my world.'\n\nx = txt.index('welcome')\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, welcome to my world.'\n\nx = txt.index('welcome')\n\nprint(x)",
    },
    {
      label: "string.index=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, And Welcome To My World!'\n\nx = txt.index('e')\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, And Welcome To My World!'\n\nx = txt.index('e')\n\nprint(x)",
    },
    {
      label: "string.index=>_3",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, And Welcome To My World!'\n\nx = txt.index('e', 5, 10)\n\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, And Welcome To My World!'\n\nx = txt.index('e', 5, 10)\n\nprint(x)",
    },
    {
      label: "string.index=>_4",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, And Welcome To My World!'\n\nprint(txt.find('q'))\nprint(txt.index('q'))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, And Welcome To My World!'\n\nprint(txt.find('q'))\nprint(txt.index('q'))",
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
      label: "string.isalnum=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'Company12'\nx = txt.isalnum()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = 'Company12'\nx = txt.isalnum()\nprint(x)",
    },
    {
      label: "string.isalnum=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'Company 12'\nx = txt.isalnum()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = 'Company 12'\nx = txt.isalnum()\nprint(x)",
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
      label: "string.isalpha=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'Company10'\nx = txt.isalpha()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = 'Company10'\nx = txt.isalpha()\nprint(x)",
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
      label: "string.isdecimal=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = '3' #unicode for 3\nx = txt.isdecimal()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = '3' #unicode for 3\nx = txt.isdecimal()\nprint(x)",
    },
    {
      label: "string.isdecimal=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = '0' #unicode for 0\nb = 'G' #unicode for G\nprint(a.isdecimal())\nprint(b.isdecimal())",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = '0' #unicode for 0\nb = 'G' #unicode for G\nprint(a.isdecimal())\nprint(b.isdecimal())",
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
      label: "string.isdigit=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = '50800'\nx = txt.isdigit()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = '50800'\nx = txt.isdigit()\nprint(x)",
    },
    {
      label: "string.isdigit=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = '0' #unicode for 0\nb = '²' #unicode for ²\nprint(a.isdigit())\nprint(b.isdigit())",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = '0' #unicode for 0\nb = '²' #unicode for ²\nprint(a.isdigit())\nprint(b.isdigit())",
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
      label: "string.isidentifier=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'Demo'\nx = txt.isidentifier()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = 'Demo'\nx = txt.isidentifier()\nprint(x)",
    },
    {
      label: "string.isidentifier=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = 'MyFolder'\nb = 'Demo002'\nc = '2bring'\nd = 'my demo'\nprint(a.isidentifier())\nprint(b.isidentifier())\nprint(c.isidentifier())\nprint(d.isidentifier())",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = 'MyFolder'\nb = 'Demo002'\nc = '2bring'\nd = 'my demo'\nprint(a.isidentifier())\nprint(b.isidentifier())\nprint(c.isidentifier())\nprint(d.isidentifier())",
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
      label: "string.islower=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'hello world!'\nx = txt.islower()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = 'hello world!'\nx = txt.islower()\nprint(x)",
    },
    {
      label: "string.islower=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = 'Hello world!'\nb = 'hello 123'\nc = 'mynameisPeter'\nprint(a.islower())\nprint(b.islower())\nprint(c.islower())",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = 'Hello world!'\nb = 'hello 123'\nc = 'mynameisPeter'\nprint(a.islower())\nprint(b.islower())\nprint(c.islower())",
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
      label: "string.isnumeric=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = '565543'\nx = txt.isnumeric()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = '565543'\nx = txt.isnumeric()\nprint(x)",
    },
    {
      label: "string.isnumeric=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = '0' #unicode for 0\nb = '²' #unicode for &sup2;\nc = '10km2'\nprint(a.isnumeric())\nprint(b.isnumeric())\nprint(c.isnumeric())",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = '0' #unicode for 0\nb = '²' #unicode for &sup2;\nc = '10km2'\nprint(a.isnumeric())\nprint(b.isnumeric())\nprint(c.isnumeric())",
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
      label: "string.isprintable=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'Hello! Are you #1?'\nx = txt.isprintable()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello! Are you #1?'\nx = txt.isprintable()\nprint(x)",
    },
    {
      label: "string.isprintable=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello!\nAre you #1?'\nx = txt.isprintable()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello!\nAre you #1?'\nx = txt.isprintable()\nprint(x)",
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
      label: "string.isspace=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = '   '\nx = txt.isspace()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = '   '\nx = txt.isspace()\nprint(x)",
    },
    {
      label: "string.isspace=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = '   s   '\nx = txt.isspace()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = '   s   '\nx = txt.isspace()\nprint(x)",
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
      label: "string.istitle=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, And Welcome To My World!'\nx = txt.istitle()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, And Welcome To My World!'\nx = txt.istitle()\nprint(x)",
    },
    {
      label: "string.istitle=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = 'HELLO, AND WELCOME TO MY WORLD'\nb = 'Hello'\nc = '22 Names'\nd = 'This Is %'!?'\nprint(a.istitle())\nprint(b.istitle())\nprint(c.istitle())\nprint(d.istitle())",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = 'HELLO, AND WELCOME TO MY WORLD'\nb = 'Hello'\nc = '22 Names'\nd = 'This Is %'!?'\nprint(a.istitle())\nprint(b.istitle())\nprint(c.istitle())\nprint(d.istitle())",
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
      label: "string.isupper=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'THIS IS NOW!'\nx = txt.isupper()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = 'THIS IS NOW!'\nx = txt.isupper()\nprint(x)",
    },
    {
      label: "string.isupper=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = 'Hello World!'\nb = 'hello 123'\nc = 'MY NAME IS PETER'\nprint(a.isupper())\nprint(b.isupper())\nprint(c.isupper())",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = 'Hello World!'\nb = 'hello 123'\nc = 'MY NAME IS PETER'\nprint(a.isupper())\nprint(b.isupper())\nprint(c.isupper())",
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
      label: "string.join=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "myTuple = ('John', 'Peter', 'Vicky')\nx = '#'.join(myTuple)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "myTuple = ('John', 'Peter', 'Vicky')\nx = '#'.join(myTuple)\nprint(x)",
    },
    {
      label: "string.join=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "myDict = {'name': 'John', 'country': 'Norway'}\nmySeparator = 'TEST'\nx = mySeparator.join(myDict)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "myDict = {'name': 'John', 'country': 'Norway'}\nmySeparator = 'TEST'\nx = mySeparator.join(myDict)\nprint(x)",
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
      label: "string.ljust=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'banana'\nx = txt.ljust(20)\nprint(x, 'is my favorite fruit.')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'banana'\nx = txt.ljust(20)\nprint(x, 'is my favorite fruit.')",
    },
    {
      label: "string.ljust=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'banana'\nx = txt.ljust(20, 'O')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = 'banana'\nx = txt.ljust(20, 'O')\nprint(x)",
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
      label: "string.lower=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'Hello my FRIENDS'\nx = txt.lower()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = 'Hello my FRIENDS'\nx = txt.lower()\nprint(x)",
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
      label: "string.lstrip=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = '     banana     '\nx = txt.lstrip()\nprint('of all fruits', x, 'is my favorite')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = '     banana     '\nx = txt.lstrip()\nprint('of all fruits', x, 'is my favorite')",
    },
    {
      label: "string.lstrip=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = ',,,,,ssaaww.....banana'\nx = txt.lstrip(',.asw')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = ',,,,,ssaaww.....banana'\nx = txt.lstrip(',.asw')\nprint(x)",
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
      label: "string.maketrans=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "# example dictionary\ndict = {'a': '123', 'b': '456', 'c': '789'}\nstring = 'abc'\nprint(string.maketrans(dict))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "# example dictionary\ndict = {'a': '123', 'b': '456', 'c': '789'}\nstring = 'abc'\nprint(string.maketrans(dict))",
    },
    {
      label: "string.maketrans=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "# example dictionary\ndict = {97: '123', 98: '456', 99: '789'}\nstring = 'abc'\nprint(string.maketrans(dict))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "# example dictionary\ndict = {97: '123', 98: '456', 99: '789'}\nstring = 'abc'\nprint(string.maketrans(dict))",
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
      label: "string.partition=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'I could eat bananas all day'\nx = txt.partition('bananas')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'I could eat bananas all day'\nx = txt.partition('bananas')\nprint(x)",
    },
    {
      label: "string.partition=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'I could eat bananas all day'\nx = txt.partition('apples')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'I could eat bananas all day'\nx = txt.partition('apples')\nprint(x)",
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
      label: "string.replace=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt 'I like bananas'\nx = txt.replace('bananas', 'apples')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt 'I like bananas'\nx = txt.replace('bananas', 'apples')\nprint(x)",
    },
    {
      label: "string.replace=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'one one was a race horse, two two was one too.'\nx = txt.replace('one', 'three')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'one one was a race horse, two two was one too.'\nx = txt.replace('one', 'three')\nprint(x)",
    },
    {
      label: "string.replace=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'one one was a race horse, two two was one too.'\nx = txt.replace('one', 'three', 2)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'one one was a race horse, two two was one too.'\nx = txt.replace('one', 'three', 2)\nprint(x)",
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
      label: "string.rfind=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'Mi casa, su casa.'\nx = txt.rfind('casa')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Mi casa, su casa.'\nx = txt.rfind('casa')\nprint(x)",
    },
    {
      label: "string.rfind=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, welcome to my world.'\nx = txt.rfind('e')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, welcome to my world.'\nx = txt.rfind('e')\nprint(x)",
    },
    {
      label: "string.rfind=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, welcome to my world.'\nx = txt.rfind('e', 5, 10)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, welcome to my world.'\nx = txt.rfind('e', 5, 10)\nprint(x)",
    },
    {
      label: "string.rfind=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, welcome to my world.'\nprint(txt.rfind('q'))\nprint(txt.rindex('q'))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, welcome to my world.'\nprint(txt.rfind('q'))\nprint(txt.rindex('q'))",
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
      label: "string.rindex=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'Mi casa, su casa.'\nx = txt.rindex('casa')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Mi casa, su casa.'\nx = txt.rindex('casa')\nprint(x)",
    },
    {
      label: "string.rindex=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, welcome to my world.'\nx = txt.rindex('e')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, welcome to my world.'\nx = txt.rindex('e')\nprint(x)",
    },
    {
      label: "string.rindex=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, welcome to my world.'\nx = txt.rindex('e', 5, 10)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, welcome to my world.'\nx = txt.rindex('e', 5, 10)\nprint(x)",
    },
    {
      label: "string.rindex=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, welcome to my world.'\nprint(txt.rfind('q'))\nprint(txt.rindex('q'))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, welcome to my world.'\nprint(txt.rfind('q'))\nprint(txt.rindex('q'))",
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
      label: "string.rpartition=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'I could eat bananas all day, bananas are my favorite fruit'\nx = txt.rpartition('bananas')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'I could eat bananas all day, bananas are my favorite fruit'\nx = txt.rpartition('bananas')\nprint(x)",
    },
    {
      label: "string.rpartition=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'I could eat bananas all day, bananas are my favorite fruit'\nx = txt.rpartition('apples')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'I could eat bananas all day, bananas are my favorite fruit'\nx = txt.rpartition('apples')\nprint(x)",
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
      label: "string.rsplit=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'apple, banana, cherry'\nx = txt.rsplit(', ')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'apple, banana, cherry'\nx = txt.rsplit(', ')\nprint(x)",
    },
    {
      label: "string.rsplit=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'apple, banana, cherry'\n# setting the max parameter to 1, will return a list with 2 elements!\nx = txt.rsplit(', ', 1)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'apple, banana, cherry'\n# setting the max parameter to 1, will return a list with 2 elements!\nx = txt.rsplit(', ', 1)\nprint(x)",
    },
    {
      label: "string.rsplit=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'banana,,,,,ssaaww.....'\nx = txt.rstrip(',.asw')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'banana,,,,,ssaaww.....'\nx = txt.rstrip(',.asw')\nprint(x)",
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
      label: "string.split=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'welcome to the jungle'\nx = txt.split()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = 'welcome to the jungle'\nx = txt.split()\nprint(x)",
    },
    {
      label: "string.split=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'hello, my name is Peter, I am 26 years old'\nx = txt.split(', ')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'hello, my name is Peter, I am 26 years old'\nx = txt.split(', ')\nprint(x)",
    },
    {
      label: "string.split=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'apple#banana#cherry#orange'\nx = txt.split('#')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'apple#banana#cherry#orange'\nx = txt.split('#')\nprint(x)",
    },
    {
      label: "string.split=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'apple#banana#cherry#orange'\n# setting the max parameter to 1, will return a list with 2 elements!\nx = txt.split('#', 1)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'apple#banana#cherry#orange'\n# setting the max parameter to 1, will return a list with 2 elements!\nx = txt.split('#', 1)\nprint(x)",
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
      label: "string.startswith=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, welcome to my world.'\nx = txt.startswith('Hello')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, welcome to my world.'\nx = txt.startswith('Hello')\nprint(x)",
    },
    {
      label: "string.startswith=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello, welcome to my world.'\nx = txt.startswith('wel', 7, 20)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello, welcome to my world.'\nx = txt.startswith('wel', 7, 20)\nprint(x)",
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
      label: "string.swapcase=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "txt = 'Hello My Name Is PETER'\nx = txt.swapcase()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Hello My Name Is PETER'\nx = txt.swapcase()\nprint(x)",
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
      label: "string.title=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'Welcome to my world'\nx = txt.title()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = 'Welcome to my world'\nx = txt.title()\nprint(x)",
    },
    {
      label: "string.title=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'Welcome to my 2nd world'\nx = txt.title()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'Welcome to my 2nd world'\nx = txt.title()\nprint(x)",
    },
    {
      label: "string.title=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = 'hello b2b2b2 and 3g3g3g'\nx = txt.title()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "txt = 'hello b2b2b2 and 3g3g3g'\nx = txt.title()\nprint(x)",
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
      label: "string.translate=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "# translation table - a dictionary\ntranslation = {97: None, 98: None, 99: 105}\n\nstring = 'abcdef'\nprint('Original string:', string)\n\n# translate string\nprint('Translated string:', string.translate(translation))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "# translation table - a dictionary\ntranslation = {97: None, 98: None, 99: 105}\n\nstring = 'abcdef'\nprint('Original string:', string)\n\n# translate string\nprint('Translated string:', string.translate(translation))",
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
      label: "string.zfill=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "txt = '50'\nx = txt.zfill(10)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "txt = '50'\nx = txt.zfill(10)\nprint(x)",
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
      label: "list.append=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = ['apple', 'banana', 'cherry']\nfruits.append('orange')\nprint(fruits)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = ['apple', 'banana', 'cherry']\nfruits.append('orange')\nprint(fruits)",
    },
    {
      label: "list.append=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "a = ['apple', 'banana', 'cherry']\nb = ['Ford', 'BMW', 'Volvo']\na.append(b)\nprint(a)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "a = ['apple', 'banana', 'cherry']\nb = ['Ford', 'BMW', 'Volvo']\na.append(b)\nprint(a)",
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
      label: "list.clear=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = ['apple', 'banana', 'cherry']\nfruits.clear()\nprint(fruits)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = ['apple', 'banana', 'cherry']\nfruits.clear()\nprint(fruits)",
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
      label: "list.copy=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = ['apple', 'banana', 'cherry']\nx = fruits.copy()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = ['apple', 'banana', 'cherry']\nx = fruits.copy()\nprint(x)",
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
      label: "list.count=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = ['apple', 'banana', 'cherry']\nx = fruits.count('cherry')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = ['apple', 'banana', 'cherry']\nx = fruits.count('cherry')\nprint(x)",
    },
    {
      label: "list.count=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = [1, 4, 2, 9, 7, 8, 9, 3, 1]\nx = fruits.count(9)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = [1, 4, 2, 9, 7, 8, 9, 3, 1]\nx = fruits.count(9)\nprint(x)",
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
      label: "list.extend=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = ['apple', 'banana', 'cherry']\npoints = (1, 4, 5, 9)\nfruits.extend(points)\nprint(fruits)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = ['apple', 'banana', 'cherry']\npoints = (1, 4, 5, 9)\nfruits.extend(points)\nprint(fruits)",
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
      label: "list.index=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = ['apple', 'banana', 'cherry']\nx = fruits.index('cherry')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = ['apple', 'banana', 'cherry']\nx = fruits.index('cherry')\nprint(x)",
    },
    {
      label: "list.index=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = [4, 55, 64, 32, 16, 32]\nx = fruits.index(32)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = [4, 55, 64, 32, 16, 32]\nx = fruits.index(32)\nprint(x)",
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
      label: "list.insert=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = ['apple', 'banana', 'cherry']\nx = fruits.insert(1, 'orange')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = ['apple', 'banana', 'cherry']\nx = fruits.insert(1, 'orange')\nprint(x)",
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
      label: "list.pop=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = ['apple', 'banana', 'cherry']\nfruits.pop(1)\nprint(fruits)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = ['apple', 'banana', 'cherry']\nfruits.pop(1)\nprint(fruits)",
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
      label: "list.remove=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = ['apple', 'banana', 'cherry']\nfruits.remove('banana')\nprint(fruits)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = ['apple', 'banana', 'cherry']\nfruits.remove('banana')\nprint(fruits)",
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
      label: "list.reverse=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = ['apple', 'banana', 'cherry']\nfruits.reverse()\nprint(fruits)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = ['apple', 'banana', 'cherry']\nfruits.reverse()\nprint(fruits)",
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
      label: "list.sort=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "cars = ['Ford', 'BMW', 'Volvo']\ncars.sort()\nprint(cars)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "cars = ['Ford', 'BMW', 'Volvo']\ncars.sort()\nprint(cars)",
    },
    {
      label: "list.sort=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "cars = ['Ford', 'BMW', 'Volvo']\ncars.sort(reverse=True)\nprint(cars)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "cars = ['Ford', 'BMW', 'Volvo']\ncars.sort(reverse=True)\nprint(cars)",
    },
    {
      label: "list.sort=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "# A function that returns the length of the value:\ndef myFunc(e):\n  return len(e)\ncars = ['Ford', 'Mitsubishi', 'BMW', 'VW']\ncars.sort(key=myFunc)\nprint(cars)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "# A function that returns the length of the value:\ndef myFunc(e):\n  return len(e)\ncars = ['Ford', 'Mitsubishi', 'BMW', 'VW']\ncars.sort(key=myFunc)\nprint(cars)",
    },
    {
      label: "list.sort=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "# A function that returns the length of the value:\ndef myFunc(e):\n  return len(e)\ncars = ['Ford', 'Mitsubishi', 'BMW', 'VW']\ncars.sort(reverse=True, key=myFunc)\nprint(cars)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "# A function that returns the length of the value:\ndef myFunc(e):\n  return len(e)\ncars = ['Ford', 'Mitsubishi', 'BMW', 'VW']\ncars.sort(reverse=True, key=myFunc)\nprint(cars)",
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
      label: "list.comp=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = [i for i in range(10)]\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = [i for i in range(10)]\nprint(x)",
    },
    {
      label: "list.comp=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = [x**2 for x in range(10)]\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = [x**2 for x in range(10)]\nprint(x)",
    },
    {
      label: "list.comp=>_3",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "list1 = [3,4,5]\nmultiplied = [item*3 for item in list1]\nprint(multiplied)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "list1 = [3,4,5]\nmultiplied = [item*3 for item in list1]\nprint(multiplied)",
    },
    {
      label: "list.comp=>_4",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "listOfWords = ['this','is','a','list','of','words']\nitems = [ word[0] for word in listOfWords ]\nprint(items)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "listOfWords = ['this','is','a','list','of','words']\nitems = [ word[0] for word in listOfWords ]\nprint(items)",
    },
    {
      label: "list.comp=>_5",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText: "x = [double(x) for x in range(10) if x%2==0]\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation: "x = [double(x) for x in range(10) if x%2==0]\nprint(x)",
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
      label: "dictionary.clear=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\ncar.clear()\nprint(car)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\ncar.clear()\nprint(car)",
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
      label: "dictionary.copy=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.copy()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.copy()\nprint(x)",
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
      label: "dictionary.fromkeys=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = ('key1', 'key2', 'key3')\ny = 0\nthisdict = dict.fromkeys(x, y)\nprint(thisdict)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = ('key1', 'key2', 'key3')\ny = 0\nthisdict = dict.fromkeys(x, y)\nprint(thisdict)",
    },
    {
      label: "dictionary.fromkeys=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = ('key1', 'key2', 'key3')\nthisdict = dict.fromkeys(x)\nprint(thisdict)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = ('key1', 'key2', 'key3')\nthisdict = dict.fromkeys(x)\nprint(thisdict)",
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
      label: "dictionary.get=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.get('model')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.get('model')\nprint(x)",
    },
    {
      label: "dictionary.get=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.get('price', 15000)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.get('price', 15000)\nprint(x)",
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
      label: "dictionary.items=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.items()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.items()\nprint(x)",
    },
    {
      label: "dictionary.items=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.items()\ncar['year'] = 2018\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.items()\ncar['year'] = 2018\nprint(x)",
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
      label: "dictionary.keys=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.keys()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.keys()\nprint(x)",
    },
    {
      label: "dictionary.keys=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.keys()\ncar['color'] = 'white'\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.keys()\ncar['color'] = 'white'\nprint(x)",
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
      label: "dictionary.pop=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\ncar.pop('model')\nprint(car)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\ncar.pop('model')\nprint(car)",
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
      label: "dictionary.popitem=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\ncar.popitem()\nprint(car)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\ncar.popitem()\nprint(car)",
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
      label: "dictionary.setdefault=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.setdefault('model', 'Bronco')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.setdefault('model', 'Bronco')\nprint(x)",
    },
    {
      label: "dictionary.setdefault=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.setdefault('color', 'white')\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.setdefault('color', 'white')\nprint(x)",
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
      label: "dictionary.update=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\ncar.update({'color': 'White'})\nprint(car)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\ncar.update({'color': 'White'})\nprint(car)",
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
      label: "dictionary.values=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.values()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.values()\nprint(x)",
    },
    {
      label: "dictionary.values=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.values()\ncar['year'] = 2018\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "car = {\n  'brand': 'Ford',\n  'model': 'Mustang',\n  'year': 1964\n}\nx = car.values()\ncar['year'] = 2018\nprint(x)",
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
      label: "tuple.count=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "thistuple = (1, 3, 7, 8, 7, 5, 4, 6, 8, 5)\nx = thistuple.count(5)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "thistuple = (1, 3, 7, 8, 7, 5, 4, 6, 8, 5)\nx = thistuple.count(5)\nprint(x)",
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
      label: "tuple.index=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "thistuple = (1, 3, 7, 8, 7, 5, 4, 6, 8, 5)\nx = thistuple.index(8)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "thistuple = (1, 3, 7, 8, 7, 5, 4, 6, 8, 5)\nx = thistuple.index(8)\nprint(x)",
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
      label: "sets.add=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = {'apple', 'banana', 'cherry'}\nfruits.add('orange') \nprint(fruits)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = {'apple', 'banana', 'cherry'}\nfruits.add('orange') \nprint(fruits)",
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
      label: "sets.clear=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = {'apple', 'banana', 'cherry'}\nfruits.clear()\nprint(fruits)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = {'apple', 'banana', 'cherry'}\nfruits.clear()\nprint(fruits)",
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
      label: "sets.copy=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = {'apple', 'banana', 'cherry'}\nx = fruits.copy()\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = {'apple', 'banana', 'cherry'}\nx = fruits.copy()\nprint(x)",
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
      label: "sets.difference=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nz = x.difference(y)\nprint(z)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nz = x.difference(y)\nprint(z)",
    },
    {
      label: "sets.difference=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nz = y.difference(x) \nprint(z)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nz = y.difference(x) \nprint(z)",
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
      label: "sets.difference_update=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nx.difference_update(y)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nx.difference_update(y)\nprint(x)",
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
      label: "sets.discard=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = {'apple', 'banana', 'cherry'}\nfruits.discard('banana') \nprint(fruits)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = {'apple', 'banana', 'cherry'}\nfruits.discard('banana') \nprint(fruits)",
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
      label: "sets.intersection=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nz = x.intersection(y)\nprint(z)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nz = x.intersection(y)\nprint(z)",
    },
    {
      label: "sets.intersection=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'a', 'b', 'c'}\ny = {'c', 'd', 'e'}\nz = {'f', 'g', 'c'}\nresult = x.intersection(y, z)\nprint(result)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'a', 'b', 'c'}\ny = {'c', 'd', 'e'}\nz = {'f', 'g', 'c'}\nresult = x.intersection(y, z)\nprint(result)",
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
      label: "sets.intersection_update=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nx.intersection_update(y)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nx.intersection_update(y)\nprint(x)",
    },
    {
      label: "sets.intersection_update=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'a', 'b', 'c'}\ny = {'c', 'd', 'e'}\nz = {'f', 'g', 'c'}\nx.intersection_update(y, z)\nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'a', 'b', 'c'}\ny = {'c', 'd', 'e'}\nz = {'f', 'g', 'c'}\nx.intersection_update(y, z)\nprint(x)",
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
      label: "sets.isdisjoint=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'facebook'}\nz = \nprint(z)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'facebook'}\nz = \nprint(z)",
    },
    {
      label: "sets.isdisjoint=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nz = x.isdisjoint(y) \nprint(z)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nz = x.isdisjoint(y) \nprint(z)",
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
      label: "sets.issubset=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'a', 'b', 'c'}\ny = {'f', 'e', 'd', 'c', 'b', 'a'}\nz = x.issubset(y) \nprint(z)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'a', 'b', 'c'}\ny = {'f', 'e', 'd', 'c', 'b', 'a'}\nz = x.issubset(y) \nprint(z)",
    },
    {
      label: "sets.issubset=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'a', 'b', 'c'}\ny = {'f', 'e', 'd', 'c', 'b'}\nz = x.issubset(y) \nprint(z)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'a', 'b', 'c'}\ny = {'f', 'e', 'd', 'c', 'b'}\nz = x.issubset(y) \nprint(z)",
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
      label: "sets.issuperset=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'f', 'e', 'd', 'c', 'b', 'a'}\ny = {'a', 'b', 'c'}\nz = x.issuperset(y) \nprint(z)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'f', 'e', 'd', 'c', 'b', 'a'}\ny = {'a', 'b', 'c'}\nz = x.issuperset(y) \nprint(z)",
    },
    {
      label: "sets.issuperset=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'f', 'e', 'd', 'c', 'b'}\ny = {'a', 'b', 'c'}\nz = x.issuperset(y) \nprint(z)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'f', 'e', 'd', 'c', 'b'}\ny = {'a', 'b', 'c'}\nz = x.issuperset(y) \nprint(z)",
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
      label: "sets.pop=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = {'apple', 'banana', 'cherry'}\nfruits.pop() \nprint(fruits)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = {'apple', 'banana', 'cherry'}\nfruits.pop() \nprint(fruits)",
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
      label: "sets.remove=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "fruits = {'apple', 'banana', 'cherry'}\nfruits.remove('banana') \nprint(fruits)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "fruits = {'apple', 'banana', 'cherry'}\nfruits.remove('banana') \nprint(fruits)",
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
      label: "sets.symmetric_difference=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nz = x.symmetric_difference(y) \nprint(z)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nz = x.symmetric_difference(y) \nprint(z)",
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
      label: "sets.symmetric_difference_update=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nx.symmetric_difference_update(y) \nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nx.symmetric_difference_update(y) \nprint(x)",
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
      label: "sets.union=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nz = x.union(y) \nprint(z)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nz = x.union(y) \nprint(z)",
    },
    {
      label: "sets.union=>_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'a', 'b', 'c'}\ny = {'f', 'd', 'a'}\nz = {'c', 'd', 'e'}\nresult = x.union(y, z) \nprint(result)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'a', 'b', 'c'}\ny = {'f', 'd', 'a'}\nz = {'c', 'd', 'e'}\nresult = x.union(y, z) \nprint(result)",
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
      label: "sets.update=>",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nx.update(y) \nprint(x)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "x = {'apple', 'banana', 'cherry'}\ny = {'google', 'microsoft', 'apple'}\nx.update(y) \nprint(x)",
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
    {
      label: "class=>_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Person:\n    pass  # An empty block\np = Person()\nprint(p)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Person:\n    pass  # An empty block\np = Person()\nprint(p)",
    },
    {
      label: "class=>inheritance_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Bird:\n\n   def __init__(self):\n     print('Bird is ready')\n\n   def whoisThis(self):\n     print('Bird')\n\n   def swim(self):\n     print('Swim faster')\n\n# child class\nclass Penguin(Bird):\n\n   def __init__(self):\n     # call super() function\n     super().__init__()\n     print('Penguin is ready')\n\n   def whoisThis(self):\n     print('Penguin')\n\n   def run(self):\n     print('Run faster')\n\npeggy = Penguin()\npeggy.whoisThis()\npeggy.swim()\npeggy.run()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Bird:\n\n   def __init__(self):\n     print('Bird is ready')\n\n   def whoisThis(self):\n     print('Bird')\n\n   def swim(self):\n     print('Swim faster')\n\n# child class\nclass Penguin(Bird):\n\n   def __init__(self):\n     # call super() function\n     super().__init__()\n     print('Penguin is ready')\n\n   def whoisThis(self):\n     print('Penguin')\n\n   def run(self):\n     print('Run faster')\n\npeggy = Penguin()\npeggy.whoisThis()\npeggy.swim()\npeggy.run()",
    },
    {
      label: "class=>inheritance_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class SchoolMember:\n    '''Represents any school member.'''\n    def __init__(self, name, age):\n        self.name = name\n        self.age = age\n        print('(Initialized SchoolMember: {})'.format(self.name))\n    def tell(self):\n        '''Tell my details.'''\n        print('Name:{} Age:{}'.format(self.name, self.age), end=' ')\nclass Teacher(SchoolMember):\n    '''Represents a teacher.'''\n    def __init__(self, name, age, salary):\n        SchoolMember.__init__(self, name, age)\n        self.salary = salary\n        print('(Initialized Teacher: {})'.format(self.name))\n    def tell(self):\n        SchoolMember.tell(self)\n        print('Salary: {:d}'.format(self.salary))\nclass Student(SchoolMember):\n    '''Represents a student.'''\n    def __init__(self, name, age, marks):\n        SchoolMember.__init__(self, name, age)\n        self.marks = marks\n        print('(Initialized Student: {})'.format(self.name))\n    def tell(self):\n        SchoolMember.tell(self)\n        print('Marks: {:d}'.format(self.marks))\nt = Teacher('Mrs. Shrividya', 40, 30000)\ns = Student('Swaroop', 25, 75)\n# prints a blank line\nprint()\nmembers = [t, s]\nfor member in members:\n    # Works for both Teachers and Students\n    member.tell()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class SchoolMember:\n    '''Represents any school member.'''\n    def __init__(self, name, age):\n        self.name = name\n        self.age = age\n        print('(Initialized SchoolMember: {})'.format(self.name))\n    def tell(self):\n        '''Tell my details.'''\n        print('Name:{} Age:{}'.format(self.name, self.age), end=' ')\nclass Teacher(SchoolMember):\n    '''Represents a teacher.'''\n    def __init__(self, name, age, salary):\n        SchoolMember.__init__(self, name, age)\n        self.salary = salary\n        print('(Initialized Teacher: {})'.format(self.name))\n    def tell(self):\n        SchoolMember.tell(self)\n        print('Salary: {:d}'.format(self.salary))\nclass Student(SchoolMember):\n    '''Represents a student.'''\n    def __init__(self, name, age, marks):\n        SchoolMember.__init__(self, name, age)\n        self.marks = marks\n        print('(Initialized Student: {})'.format(self.name))\n    def tell(self):\n        SchoolMember.tell(self)\n        print('Marks: {:d}'.format(self.marks))\nt = Teacher('Mrs. Shrividya', 40, 30000)\ns = Student('Swaroop', 25, 75)\n# prints a blank line\nprint()\nmembers = [t, s]\nfor member in members:\n    # Works for both Teachers and Students\n    member.tell()",
    },
    {
      label: "class=>with_attribute_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Parrot:\n\n# class attribute\n species = 'bird'\n\n# instance attribute\n def __init__(self, name, age):\n    self.name = name\n    self.age = age\n\n# instantiate the Parrot class\nblu = Parrot('Blu', 10)\nwoo = Parrot('woo', 15)\n\n# access the class attributes\nprint('Blu is a {}'.format(blu.__class__.species))\nprint('Woo is also a {}'.format(woo.__class__.species))\n# access the instance attributes\nprint('{} is {} years old'.format( blu.name, blu.age))\nprint('{} is {} years old'.format( woo.name, woo.age))",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Parrot:\n\n# class attribute\n species = 'bird'\n\n# instance attribute\n def __init__(self, name, age):\n    self.name = name\n    self.age = age\n\n# instantiate the Parrot class\nblu = Parrot('Blu', 10)\nwoo = Parrot('woo', 15)\n\n# access the class attributes\nprint('Blu is a {}'.format(blu.__class__.species))\nprint('Woo is also a {}'.format(woo.__class__.species))\n# access the instance attributes\nprint('{} is {} years old'.format( blu.name, blu.age))\nprint('{} is {} years old'.format( woo.name, woo.age))",
    },
    {
      label: "class=>with_attribute_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Person:\n    def __init__(self, name):\n        self.name = name\n    def say_hi(self):\n        print('Hello, my name is', self.name)\np = Person('Swaroop')\np.say_hi()\n# The previous 2 lines can also be written as\n# Person('Swaroop').say_hi()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Person:\n    def __init__(self, name):\n        self.name = name\n    def say_hi(self):\n        print('Hello, my name is', self.name)\np = Person('Swaroop')\np.say_hi()\n# The previous 2 lines can also be written as\n# Person('Swaroop').say_hi()",
    },
    {
      label: "class=>with_attribute_3",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Robot:\n    '''Represents a robot, with a name.'''\n    # A class variable, counting the number of robots\n    population = 0\n    def __init__(self, name):\n        '''Initializes the data.'''\n        self.name = name\n        print('(Initializing {})'.format(self.name))\n        # When this person is created, the robot\n        # adds to the population\n        Robot.population += 1\n    def die(self):\n        '''I am dying.'''\n        print('{} is being destroyed!'.format(self.name))\n        Robot.population -= 1\n        if Robot.population == 0:\n            print('{} was the last one.'.format(self.name))\n        else:\n            print('There are still {:d} robots working.'.format(\n                Robot.population))\n    def say_hi(self):\n        '''Greeting by the robot.\n        Yeah, they can do that.'''\n        print('Greetings, my masters call me {}.'.format(self.name))\n    @classmethod\n    def how_many(cls):\n        '''Prints the current population.'''\n        print('We have {:d} robots.'.format(cls.population))\ndroid1 = Robot('R2-D2')\ndroid1.say_hi()\nRobot.how_many()\ndroid2 = Robot('C-3PO')\ndroid2.say_hi()\nRobot.how_many()\nprint('Robots can do some work here.')\nprint('Robots have finished their work. So lets destroy them.')\ndroid1.die()\ndroid2.die()\nRobot.how_many()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Robot:\n    '''Represents a robot, with a name.'''\n    # A class variable, counting the number of robots\n    population = 0\n    def __init__(self, name):\n        '''Initializes the data.'''\n        self.name = name\n        print('(Initializing {})'.format(self.name))\n        # When this person is created, the robot\n        # adds to the population\n        Robot.population += 1\n    def die(self):\n        '''I am dying.'''\n        print('{} is being destroyed!'.format(self.name))\n        Robot.population -= 1\n        if Robot.population == 0:\n            print('{} was the last one.'.format(self.name))\n        else:\n            print('There are still {:d} robots working.'.format(\n                Robot.population))\n    def say_hi(self):\n        '''Greeting by the robot.\n        Yeah, they can do that.'''\n        print('Greetings, my masters call me {}.'.format(self.name))\n    @classmethod\n    def how_many(cls):\n        '''Prints the current population.'''\n        print('We have {:d} robots.'.format(cls.population))\ndroid1 = Robot('R2-D2')\ndroid1.say_hi()\nRobot.how_many()\ndroid2 = Robot('C-3PO')\ndroid2.say_hi()\nRobot.how_many()\nprint('Robots can do some work here.')\nprint('Robots have finished their work. So lets destroy them.')\ndroid1.die()\ndroid2.die()\nRobot.how_many()",
    },
    {
      label: "class=>with_method_1",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Parrot:\n\n# instance attributes\n def __init__(self, name, age):\n   self.name = name\n   self.age = age\n\n# instance method\n def sing(self, song):\n   return '{} sings {}'.format(self.name, song)\n\n def dance(self):\n   return '{} is now dancing'.format(self.name)\n\n# instantiate the object\nblu = Parrot('Blu', 10)\n# call our instance methods\nprint(blu.sing('Happy'))\nprint(blu.dance())",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Parrot:\n\n# instance attributes\n def __init__(self, name, age):\n   self.name = name\n   self.age = age\n\n# instance method\n def sing(self, song):\n   return '{} sings {}'.format(self.name, song)\n\n def dance(self):\n   return '{} is now dancing'.format(self.name)\n\n# instantiate the object\nblu = Parrot('Blu', 10)\n# call our instance methods\nprint(blu.sing('Happy'))\nprint(blu.dance())",
    },
    {
      label: "class=>with_method_2",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Person:\n    def say_hi(self):\n        print('Hello, how are you?')\np = Person()\np.say_hi()\n# The previous 2 lines can also be written as\n# Person().say_hi()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Person:\n    def say_hi(self):\n        print('Hello, how are you?')\np = Person()\np.say_hi()\n# The previous 2 lines can also be written as\n# Person().say_hi()",
    },
    {
      label: "class=>encapsulation",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Computer:\n\n def __init__(self):\n   self.__maxprice = 900\n\n def sell(self):\n   print('Selling Price: {}'.format(self.__maxprice))\n\n def setMaxPrice(self, price):\n   self.__maxprice = price\n\nc = Computer()\nc.sell()\n\n# change the price\nc.__maxprice = 1000\nc.sell()\n\n# using setter function\nc.setMaxPrice(1000)\nc.sell()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Computer:\n\n def __init__(self):\n   self.__maxprice = 900\n\n def sell(self):\n   print('Selling Price: {}'.format(self.__maxprice))\n\n def setMaxPrice(self, price):\n   self.__maxprice = price\n\nc = Computer()\nc.sell()\n\n# change the price\nc.__maxprice = 1000\nc.sell()\n\n# using setter function\nc.setMaxPrice(1000)\nc.sell()",
    },
    {
      label: "class=>polymorphism",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Parrot:\n\n def fly(self):\n   print('Parrot can fly')\n\n def swim(self):\n   print('Parrot can not swim')\n\nclass Penguin:\n\n def fly(self):\n   print('Penguin can not fly')\n\n def swim(self):\n   print('Penguin can swim')\n\n# common interface\ndef flying_test(bird):\n  bird.fly()\n\n#instantiate objects\nblu = Parrot()\npeggy = Penguin()\n\n# passing the object\nflying_test(blu)\nflying_test(peggy)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Parrot:\n\n def fly(self):\n   print('Parrot can fly')\n\n def swim(self):\n   print('Parrot can not swim')\n\nclass Penguin:\n\n def fly(self):\n   print('Penguin can not fly')\n\n def swim(self):\n   print('Penguin can swim')\n\n# common interface\ndef flying_test(bird):\n  bird.fly()\n\n#instantiate objects\nblu = Parrot()\npeggy = Penguin()\n\n# passing the object\nflying_test(blu)\nflying_test(peggy)",
    },
    {
      label: "class=>polymorphism",
      kind: monaco.languages.CompletionItemKind.Snippet,
      insertText:
        "class Parrot:\n\n def fly(self):\n   print('Parrot can fly')\n\n def swim(self):\n   print('Parrot can not swim')\n\nclass Penguin:\n\n def fly(self):\n   print('Penguin can not fly')\n\n def swim(self):\n   print('Penguin can swim')\n\n# common interface\ndef flying_test(bird):\n  bird.fly()\n\n#instantiate objects\nblu = Parrot()\npeggy = Penguin()\n\n# passing the object\nflying_test(blu)\nflying_test(peggy)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
      documentation:
        "class Parrot:\n\n def fly(self):\n   print('Parrot can fly')\n\n def swim(self):\n   print('Parrot can not swim')\n\nclass Penguin:\n\n def fly(self):\n   print('Penguin can not fly')\n\n def swim(self):\n   print('Penguin can swim')\n\n# common interface\ndef flying_test(bird):\n  bird.fly()\n\n#instantiate objects\nblu = Parrot()\npeggy = Penguin()\n\n# passing the object\nflying_test(blu)\nflying_test(peggy)",
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
