import {
  pylint_error,
  pylint_warning,
  pylint_convention,
  pylint_refactor,
  pylint_fatal,
} from "../constants";

// post and response code format
export const fetchFormatCode = async ({
  baseUrl,
  monacoEditorSourceCode,
  setMonacoEditorSourceCode,
}) => {
  try {
    const response = await fetch(`${baseUrl}/api/v1/format/`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        code: monacoEditorSourceCode,
      }),
    });

    const data = await response.json();
    if (response.ok) {
      setMonacoEditorSourceCode(data.formatted_code);
    } else {
      console.error("Error formatting code:", data.error);
    }
  } catch (error) {
    console.log("error ", error);
  }
};

// post and response code format
export const fetchAnalysisCode = async ({
  baseUrl,
  monacoEditorSourceCode,
  controller,
}) => {
  try {
    const response = await fetch(`${baseUrl}/api/v1/analysis/`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        code: monacoEditorSourceCode,
        disable_errors: [
          ...pylint_error,
          ...pylint_warning,
          ...pylint_convention,
          ...pylint_refactor,
          ...pylint_fatal,
        ],
      }),
      signal: controller.signal,
    });

    const data = await response.json();

    if (response.ok) {
      return data;
    } else {
      console.error("Error formatting code:", data.error);
    }
  } catch (error) {
    if (error.name !== "AbortError") {
      console.log(error);
    }
  }
};

export const getMarkerSeverity = ({ type, monaco }) => {
  switch (type) {
    case "refactor":
    case "convention":
      return monaco.MarkerSeverity.Info;
    case "error":
      return monaco.MarkerSeverity.Error;
    case "warning":
    case "fatal":
      return monaco.MarkerSeverity.Warning;
    default:
      return monaco.MarkerSeverity.Error;
  }
};

export const getHalMethods = ({ monaco }) => {
  return [
    {
      label: "showImage(image)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "showImage(image)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "allows you to view a debug image or with relevant information.",
    },
    {
      label: "showLeftImage(cv2_image)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "showLeftImage(cv2_image)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "Shows another image of the camera in the GUI.",
    },
    {
      label: "getPose3d()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getPose3d()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "returns x,y and theta components of the robot in world coordinates.",
    },
    {
      label: "getPose3d().x",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getPose3d().x",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "get the position of the robot (x coordinate).",
    },
    {
      label: "getPose3d().y",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getPose3d().y",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "obtain the position of the robot (y coordinate).",
    },
    {
      label: "getPose3d().yaw",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getPose3d().yaw",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "get the orientation of the robot with regarding the map.",
    },
    {
      label: "getLaserData()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getLaserData()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "obtain laser sensor data It is composed of 180 pairs of values: (0-180º distance in meters).",
    },
    {
      label: "setV()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "setV()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "set the linear speed.",
    },
    {
      label: "setW()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "setW()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "set the angular velocity.",
    },
    {
      label: "getBumperData().state",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getBumperData().state",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        " To establish if the robot has crashed or not. Returns a 1 if the robot collides and a 0 if it has not crashed.",
    },
    {
      label: "getBumperData().bumper",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getBumperData().bumper",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "If the robot has crashed, it turns to 1 when the crash occurs at the center of the robot, 0 when it occurs at its right and 2 if the collision is at its left.",
    },
    {
      label: "getImage('left')",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getImage('left')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "get the left image.",
    },
    {
      label: "getImage('right')",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getImage('right')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "get the right image.",
    },
    {
      label: "getCameraPosition('left')",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getCameraPosition('left')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "to get the left camera position from ROS Driver Camera.",
    },
    {
      label: "getCameraPosition('right')",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getCameraPosition('right')",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "get the right camera position from ROS Driver Camera.",
    },
    {
      label: "graficToOptical('left', point2d)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "graficToOptical('left', point2d)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "transform the Image Coordinate System to the Camera System.",
    },

    {
      label: "backproject('left', point2d)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "backproject('left', point2d)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "backprojects the 2D Image Point into 3D Point Space.",
    },
    {
      label: "project('left', point3d)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "project('left', point3d)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "backprojects a 3D Point Space into the 2D Image Point.",
    },
    {
      label: "opticalToGrafic('left', point2d)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "opticalToGrafic('left', point2d)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "transform the Camera System to the Image Coordinate System.",
    },
    {
      label: "project3DScene(point3d)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "project3DScene(point3d)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "transform 3D Point Space after triangulation to the 3D Point Viewer.",
    },
    {
      label: "get_position()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "get_position()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "Returns the actual position of the drone as a numpy array [x, y, z], in m.",
    },
    {
      label: "get_velocity()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "get_velocity()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "Returns the actual velocities of the drone as a numpy array [vx, vy, vz], in m/s.",
    },
    {
      label: "get_yaw_rate()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "get_yaw_rate()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: " Returns the actual yaw rate of the drone, in rad/s.",
    },
    {
      label: "get_yaw()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "get_yaw()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "Returns the yaw angle of the drone, in rad.",
    },
    {
      label: "get_orientation()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "get_orientation()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        " Returns the actual roll, pitch and yaw of the drone as a numpy array [roll, pitch, yaw], in rad.",
    },
    {
      label: "get_roll()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "get_roll()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "Returns the roll angle of the drone, in rad.",
    },
    {
      label: "get_pitch()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "get_pitch()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "Returns the pitch angle of the drone, in rad.",
    },
    {
      label: "get_yaw()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "get_yaw()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "Returns the yaw angle of the drone, in rad.",
    },
    {
      label: "get_landed_state()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "get_landed_state()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "Returns 1 if the drone is on the ground (landed), 2 if the drone is in the air and 4 if the drone is landing. 0 could be also returned if the drone landed state is unknown..",
    },
    {
      label: "set_cmd_pos(x, y, z, az)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "set_cmd_pos(x, y, z, az)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "Commands the position (x,y,z) of the drone, in m and the yaw angle (az) (in rad) taking as reference the first takeoff point (map frame).",
    },

    {
      label: "set_cmd_vel(vx, vy, vz, az)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "set_cmd_vel(vx, vy, vz, az)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "Commands the linear velocity of the drone in the x, y and z directions (in m/s) and the yaw rate (az) (rad/s) in its body fixed frame.",
    },
    {
      label: "set_cmd_mix(vx, vy, z, az)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "set_cmd_mix(vx, vy, z, az)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "Commands the linear velocity of the drone in the x, y directions (in m/s), the height (z) related to the takeoff point and the yaw rate (az) (in rad/s).",
    },
    {
      label: "takeoff(height)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "takeoff(height)",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "Takeoff at the current location, to the given height (in m).",
    },
    {
      label: "land()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "land()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "Land at the current location.",
    },
    {
      label: "get_frontal_image()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "get_frontal_image()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "Returns the latest image from the frontal camera as a OpenCV cv2_image.",
    },
    {
      label: "get_ventral_image()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "get_ventral_image()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        " Returns the latest image from the ventral camera as a OpenCV cv2_image.",
    },
    {
      label: "lift()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "lift()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "lift the platform.",
    },

    {
      label: "putdown()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "putdown()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: "put down the platform.",
    },
    {
      label: "getBoundingBoxes()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getBoundingBoxes()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        " this method calls a detect() neural network's method to obtain a list of detected objects from an image passed as argument.",
    },
    {
      label: "getFrontLaserData()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getFrontLaserData()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "obtain the front laser sensor data It is composed of 180 pairs of values: (0-180º distance in millimeters).",
    },
    {
      label: "getRightLaserData()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getRightLaserData()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "obtain the right laser sensor data It is composed of 180 pairs of values: (0-180º distance in millimeters)getBackLaserData().",
    },
    {
      label: "getBackLaserData()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getBackLaserData()",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation:
        "obtain the back laser sensor data It is composed of 180 pairs of values: (0-180º distance in millimeters).",
    },
  ];
};

// gui
export const getGuiMethods = ({ monaco }) => {
  return [
    {
      label: "getImage()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getImage()",
      documentation: "get the image (BGR8).",
    },
    {
      label: "setV(velocity)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "setV(velocity)",
      documentation: "set the linear speed.",
    },
    {
      label: "setW(velocity)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "setW(velocity)",
      documentation: "set the angular velocity.",
    },
    {
      label: "getNextTarget()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getNextTarget()",
      documentation: "obtain the next target object on the scenario.",
    },
    {
      label: "setTargetx",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "setTargetx",
      documentation: "sets the x coordinate of the target on the GUI.",
    },
    {
      label: "setTargety",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "setTargety",
      documentation: "sets the y coordinate of the target on the GUI.",
    },
    {
      label: "showForces",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "showForces",
      documentation: "shows the forces being appliend on the car in real time.",
    },
    {
      label: "showNumpy(mat)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "showNumpy(mat)",
      documentation: "Displays the matrix sent.",
    },
    {
      label: "getMap(url)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getMap(url)",
      documentation:
        "Returns a numpy array with the image data in a 3 dimensional array (R, G, B, A).",
    },
    {
      label: "ShowNewPoints(points)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "ShowNewPoints(points)",
      documentation: "plot a array of plots in the 3D visor.",
    },
    {
      label: "ShowAllPoints(points)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "ShowAllPoints(points)",
      documentation: "clear the 3D visor and plot new array of plots.",
    },
    {
      label: "ClearAllPoints()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "ClearAllPoints()",
      documentation: "clear the 3D visor.",
    },
    {
      label: "showImage(cv2_image)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "showImage(cv2_image)",
      documentation: "Shows a image of the camera in the GUI.",
    },
    {
      label: "showImages(imageLeft,imageRight,True)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "showImages(imageLeft,imageRight,True)",
      documentation:
        "allows you to view a debug images or with relevant information.",
    },
    {
      label: "showImages(imageLeft,imageRight,True)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "showImages(imageLeft,imageRight,True)",
      documentation:
        "allows you to view a debug images or with relevant information.",
    },
    {
      label: "showPath(array)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "showPath(array)",
      documentation: "shows a path on the map.",
    },
    {
      label: "getTargetPose()",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "getTargetPose()",
      documentation:
        "returns x,y coordinates of chosen destionation in the world.",
    },
    {
      label: "rowColumn(vector)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "rowColumn(vector)",
      documentation:
        "returns the index in map coordinates corresponding to the vector in world coordinates passed as parameter.",
    },
    {
      label: "showParticles(particles)",
      kind: monaco.languages.CompletionItemKind.Method,
      insertText: "showParticles(particles)",
      documentation:
        "shows the particles on the map. It is necessary to pass a list of particles as an argument. Each particle must be a list with [positionx, positiony, angle].",
    },
  ];
};
