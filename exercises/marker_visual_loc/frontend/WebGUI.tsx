import React, { useState, useEffect, useRef } from "react";
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

import house from "./resources/map.png";
import "./css/GUICanvas.css";

interface Beacon {
  id: string;
  x: number;
  y: number;
  type: string;
}

function WebGUI() {
  const [realPose, setRealPose] = useState<number[] | undefined>();
  const [noisyPose, setNoisyPose] = useState<number[] | undefined>();
  const [userPose, setUserPose] = useState<number[] | undefined>();
  const [realPath, setRealPath] = useState<string>("");
  const [noisyPath, setNoisyPath] = useState<string>("");
  const [userPath, setUserPath] = useState<string>("");
  const [resizedBeacons, setResizedBeacons] = useState<Beacon[]>([]);
  const [userImage, setUserImage] = useState<string | undefined>(undefined);
  const canvasRef = useRef<HTMLImageElement>(null);
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  let realTrail: number[][] = [];
  let noisyTrail: number[][] = [];
  let userTrail: number[][] = [];
  let realLastPose: number[] | undefined = undefined;
  let noisyLastPose: number[] | undefined = undefined;
  let userLastPose: number[] | undefined = undefined;
  let valuesUntilValid = 0;

  const timeout = 0;

  const beacons: Beacon[] = [
    { id: "tag_0", x: 518.75, y: 270.325, type: "hor" },
    { id: "tag_1", x: 481.4, y: 810.775, type: "hor" },
    { id: "tag_2", x: 196.395, y: 339.15, type: "vert" },
    { id: "tag_3", x: 400.89, y: 79.9, type: "hor" },
    { id: "tag_4", x: 844.94, y: 712.3, type: "vert" },
    { id: "tag_5", x: 295.03, y: 499.8, type: "vert" },
    { id: "tag_6", x: 730.4, y: 350.55, type: "hor" },
    { id: "tag_7", x: 499.66, y: 140.25, type: "vert" },
  ];

  const resizeObserver = new ResizeObserver((entries) => {
    const img = entries[0].target;
    //or however you get a handle to the IMG
    const width = img.clientWidth / 1012;
    const height = img.clientHeight / 1012;

    setResizedBeacons(
      beacons.map((beacon) => ({
        id: beacon.id,
        x: beacon.x * width,
        y: beacon.y * height,
        type: beacon.type,
      }))
    );

    updatePath(realTrail, setRealPath, height, width);
    updatePath(noisyTrail, setNoisyPath, height, width);
    updatePath(userTrail, setUserPath, height, width);

    if (realLastPose) {
      setRealPose([
        realLastPose[1] * height,
        realLastPose[0] * width,
        -1.57 - realLastPose[2],
      ]);
    }

    if (noisyLastPose) {
      setNoisyPose([
        noisyLastPose[1] * height,
        noisyLastPose[0] * width,
        -1.57 - noisyLastPose[2],
      ]);
    }

    if (userLastPose) {
      setUserPose([
        userLastPose[1] * height,
        userLastPose[0] * width,
        -1.57 - userLastPose[2],
      ]);
    }

    valuesUntilValid = 0;
  });

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;
    const update = data.update;

    const img = canvasRef.current;
    if (img === null) {
      return;
    }
    const width = img.clientWidth / 1012;
    const height = img.clientHeight / 1012;

    if (update.real_pose) {
      const pose = update.real_pose.substring(1, update.real_pose.length - 1);
      const content = pose.split(",").map((item: string) => parseFloat(item));
      realLastPose = content;

      setRealPose([
        content[1] * height,
        content[0] * width,
        -1.57 - content[2],
      ]);
      if (valuesUntilValid > timeout) {
        updatePath(realTrail, setRealPath, height, width);
        addToPath(content[1], content[0], realTrail);
      } else {
        valuesUntilValid = valuesUntilValid + 1;
      }
    }

    if (update.noisy_pose) {
      const pose = update.noisy_pose.substring(1, update.noisy_pose.length - 1);
      const content = pose.split(",").map((item: string) => parseFloat(item));
      noisyLastPose = content;

      setNoisyPose([
        content[1] * height,
        content[0] * width,
        -1.57 - content[2],
      ]);
      if (valuesUntilValid > timeout) {
        updatePath(noisyTrail, setNoisyPath, height, width);
        addToPath(content[1], content[0], noisyTrail);
      }
    }

    if (update.estimate_pose) {
      const pose = update.estimate_pose.substring(
        1,
        update.estimate_pose.length - 1
      );
      const content = pose.split(",").map((item: string) => parseFloat(item));
      userLastPose = content;

      setUserPose([
        content[1] * height,
        content[0] * width,
        -1.57 - content[2],
      ]);
      if (valuesUntilValid > timeout) {
        updatePath(userTrail, setUserPath, height, width);
        addToPath(content[1], content[0], userTrail);
      }
    }

    if (update.image) {
      const image = JSON.parse(update.image);
      if (image.shape instanceof Array) {
        setUserImage(`data:image/png;base64,${image.image}`);
      }
    }
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setRealPose(undefined);
      setNoisyPose(undefined);
      setUserPose(undefined);
      setUserImage(undefined);
      setRealPath("");
      setNoisyPath("");
      setUserPath("");
      realTrail = [];
      noisyTrail = [];
      userTrail = [];
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
        src={house}
        style={{ left: "0" }}
      />
      <WebGUIImage src={userImage} style={{ left: "50%" }} />
      {realPose && (
        <div
          id="real-pos"
          style={{
            rotate: "z " + realPose[2] + "rad",
            top: realPose[0] - 10,
            left: realPose[1] - 5,
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
            left: noisyPose[1] - 5,
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
      {userPose && (
        <div
          id="user-pos"
          style={{
            rotate: "z " + userPose[2] + "rad",
            top: userPose[0] - 10,
            left: userPose[1] - 5,
          }}
        >
          <img src={RobotRed} id="user-pos" />
        </div>
      )}
      {userPath && (
        <svg
          height="100%"
          width="100%"
          xmlns="http://www.w3.org/2000/svg"
          style={{ zIndex: 2, position: "absolute" }}
        >
          <path
            xmlns="http://www.w3.org/2000/svg"
            d={userPath}
            style={{
              strokeWidth: "1px",
              strokeLinejoin: "round",
              stroke: "red",
              fill: "none",
              opacity: "0.5",
            }}
          />
        </svg>
      )}
      {Object.values(resizedBeacons).map((beacon) => {
        return (
          <div
            key={beacon.id}
            className={`beacon ${beacon.type}`}
            style={{
              top: `${beacon.y}px`,
              left: `${beacon.x}px`,
              position: "absolute",
              border: "2px solid rgb(255, 255, 255)",
              cursor: "pointer",
              zIndex: "5",
              width: `${beacon.type == "vert" ? 0 : 20}px`,
              height: `${beacon.type == "hor" ? 0 : 20}px`,
            }}
            title={`ID: ${beacon.id}`}
          />
        );
      })}
    </WebGUIContainer>
  );
}

export default WebGUI;
