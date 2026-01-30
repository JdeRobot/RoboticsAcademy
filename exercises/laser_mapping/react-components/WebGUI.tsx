import { useState, useEffect, useRef } from "react";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import { updatePath, addToPath } from "./helpers/showImageVisual";
import RobotRed from "./resources/robot_red.svg";
import RobotGreen from "./resources/robot_green.svg";
import RobotBlue from "./resources/robot_blue.svg";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";

import warehouse from "./resources/map.png";
import smallWarehouse from "./resources/small_map.png";

import "./css/GUICanvas.css";
function WebGUI() {
  const [realPose, setRealPose] = useState<number[] | undefined>(undefined);
  const [noisyPose, setNoisyPose] = useState<number[] | undefined>(undefined);
  const [realPath, setRealPath] = useState<string>("");
  const [noisyPath, setNoisyPath] = useState<string>("");
  const [mapImg, setMapImg] = useState(smallWarehouse);
  const [userImage, setUserImage] = useState<string | undefined>(undefined);
  const canvasRef = useRef<HTMLImageElement>(null);

  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  var realTrail: number[][] = [];
  var noisyTrail: number[][] = [];
  var realLastPose: number[] | undefined = undefined;
  var noisyLastPose: number[] | undefined = undefined;
  var valuesUntilValid: number = 0;

  const timeout = 40;

  const resizeObserver = new ResizeObserver((entries) => {
    var img = entries[0].target;
    //or however you get a handle to the IMG
    var width = img.clientWidth / 1500;
    var height = img.clientHeight / 970;

    updatePath(realTrail, setRealPath, height, width);
    updatePath(noisyTrail, setNoisyPath, height, width);

    if (realLastPose) {
      setRealPose([
        realLastPose[0] * height,
        realLastPose[1] * width,
        3.14 - realLastPose[2],
      ]);
    }

    if (noisyLastPose) {
      setNoisyPose([
        noisyLastPose[0] * height,
        noisyLastPose[1] * width,
        3.14 - noisyLastPose[2],
      ]);
    }

    valuesUntilValid = 0;
  });

  const updateCallback = (updateData: unknown) => {
    let data = updateData as any;
    const update = data.update;

    var img = canvasRef.current;
    if (img === null) {
      return;
    }
    var width = img.clientWidth / 1500;
    var height = img.clientHeight / 970;

    if (update.real_pose) {
      const pose = update.real_pose.substring(1, update.real_pose.length - 1);
      const content = pose.split(",").map((item: string) => parseFloat(item));
      realLastPose = content;

      setRealPose([content[0] * height, content[1] * width, 3.14 - content[2]]);
      if (valuesUntilValid > timeout) {
        updatePath(realTrail, setRealPath, height, width);
        addToPath(content[0], content[1], realTrail);
      } else {
        valuesUntilValid = valuesUntilValid + 1;
      }
    }

    if (update.noisy_pose) {
      const pose = update.noisy_pose.substring(1, update.noisy_pose.length - 1);
      const content = pose.split(",").map((item: string) => parseFloat(item));
      noisyLastPose = content;

      setNoisyPose([
        content[0] * height,
        content[1] * width,
        3.14 - content[2],
      ]);
      if (valuesUntilValid > timeout) {
        updatePath(noisyTrail, setNoisyPath, height, width);
        addToPath(content[0], content[1], noisyTrail);
      }
    }

    if (update.user_map) {
      let image = JSON.parse(update.user_map);
      if (image.shape instanceof Array) {
        setUserImage(`data:image/png;base64,${image.user_map}`);
      }
    }
  };

  const stateCallback = (state: string) => {
    if (manager === null) {
      return;
    }

    if (state === states.TOOLS_READY) {
      setRealPose(undefined);
      setNoisyPose(undefined);
      setRealPath("");
      setNoisyPath("");
      realTrail = [];
      noisyTrail = [];
      setUserImage(undefined);

      switch (manager.getUniverse()) {
        case "Laser Mapping Warehouse":
          setMapImg(warehouse);
          break;
        case "Small Laser Mapping Warehouse":
          setMapImg(smallWarehouse);
          break;
      }
    }

    valuesUntilValid = 0;
  };

  connectApplication(
    manager,
    updateCallback,
    stateCallback,
    canvasRef,
    resizeObserver
  );

  return (
    <WebGUIContainer>
      <WebGUIImage
        reference={canvasRef}
        id="map-img"
        src={mapImg}
        style={{ left: "0" }}
      />
      <WebGUIImage src={userImage} style={{ left: "50%" }} />
      {realPose && (
        <div
          id="real-pos"
          style={{
            rotate: "z " + realPose[2] + "rad",
            top: realPose[0] - 10,
            left: realPose[1] - 10,
          }}
        >
          <img src={RobotGreen} id="real-pos" />
        </div>
      )}
      {realPath && (
        <svg
          height="100%"
          width="100%"
          xmlns="http://www.w3.org/2000/svg"
          style={{ zIndex: 2, position: "absolute" }}
        >
          <path
            xmlns="http://www.w3.org/2000/svg"
            d={realPath}
            style={{
              strokeWidth: "1px",
              strokeLinejoin: "round",
              stroke: "green",
              fill: "none",
              opacity: "0.5",
            }}
          />
        </svg>
      )}
      {noisyPose && (
        <div
          id="noisy-pos"
          style={{
            rotate: "z " + noisyPose[2] + "rad",
            top: noisyPose[0] - 10,
            left: noisyPose[1] - 10,
          }}
        >
          <img src={RobotBlue} id="noisy-pos" />
        </div>
      )}
      {noisyPath && (
        <svg
          height="100%"
          width="100%"
          xmlns="http://www.w3.org/2000/svg"
          style={{ zIndex: 2, position: "absolute" }}
        >
          <path
            xmlns="http://www.w3.org/2000/svg"
            d={noisyPath}
            style={{
              strokeWidth: "1px",
              strokeLinejoin: "round",
              stroke: "blue",
              fill: "none",
              opacity: "0.5",
            }}
          />
        </svg>
      )}
    </WebGUIContainer>
  );
}

export default WebGUI;
