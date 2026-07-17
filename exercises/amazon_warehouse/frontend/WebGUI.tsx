import React, { useState, useEffect, useRef } from "react";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import {
  updatePath,
  addToTrail,
  updateTrail,
} from "./helpers/AmazonWarehouseHelper";

import Map1 from "./resources/map.png";
import Map2 from "./resources/map_2.png";

import "./css/GUICanvas.css";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import WebGUIImage from "Components/exercise/WebGUIImage";

function WebGUI() {
  const Map1Size = { width: 415, height: 279 };
  const Map2Size = { width: 1075, height: 699 };

  const [map, setMap] = useState<string>(Map1);
  const [mapSize, setMapSize] = useState(Map1Size);
  const [vehicleType, setVehicleType] = useState(0); // 0=normal 1=ackermann
  const [vehiclePose, setVehiclePose] = useState<number[] | undefined>(
    undefined,
  );
  const [targetPose, setTargetPose] = useState<number[] | undefined>(undefined);
  const [trail, setTrail] = useState<string>("");
  const [path, setPath] = useState<string>("");
  const [liftState, setLiftState] = useState<boolean>(false);

  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);
  const canvasRef = useRef<HTMLImageElement>(null);

  let base_path: number[][] = [];
  let base_trail: number[][] = [];
  let lastPose: number[] | undefined = undefined;

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const resizeObserver = new ResizeObserver((entries) => {
    const img = entries[0].target;
    const width = (img.clientWidth / mapSize.width) * 1.38;
    const height = (img.clientHeight / mapSize.height) * 1.9;

    updatePath(base_path, setPath, height, width);
    updateTrail(base_trail, setTrail, height, width);

    if (lastPose) {
      const ang = -lastPose[2];

      setVehiclePose([lastPose[1] * height, lastPose[0] * width, ang]);
    }
  });

  const displayRobot = (data: any) => {
    if (data.map) {
      const pose = data.map.substring(1, data.map.length - 1);
      const content = pose.split(",").map((item: string) => parseFloat(item));

      let resize_factor = 1,
        offset_x = 0,
        offset_y = 0;
      if (map == Map2) {
        resize_factor = 0.62;
        offset_x = 59;
        offset_y = 36;
      }

      const convPose = [
        content[0] * resize_factor + offset_x,
        content[1] * resize_factor + offset_y,
      ];

      lastPose = convPose;

      const img = canvasRef.current;
      if (img === null) {
        return;
      }
      //or however you get a handle to the IMG
      const width = (img.clientWidth / mapSize.width) * 1.38;
      const height = (img.clientHeight / mapSize.height) * 1.9;

      updateTrail(base_trail, setTrail, height, width);

      const ang = -content[2];

      setVehiclePose([convPose[1] * height, convPose[0] * width, ang]);
      addToTrail(convPose[1], convPose[0], base_trail);
    }

    if (data.image) {
      const image = JSON.parse(data.image);
      if (image.image != "" && image.shape instanceof Array) {
        setMap(`data:image/png;base64,${image.image}`);
      }
    }
  };

  const displayPath = (data: any) => {
    if (data.array) {
      const img = canvasRef.current;
      if (img === null) {
        return;
      }
      const width = (img.clientWidth / mapSize.width) * 1.38;
      const height = (img.clientHeight / mapSize.height) * 1.9;

      base_path = JSON.parse(data.array);
      updatePath(base_path, setPath, height, width);

      const target = base_path.at(-1);
      if (target) {
        setTargetPose([
          (target[1] * height * 100) / img.clientHeight,
          (target[0] * width * 100) / img.clientWidth,
        ]);
      }
    }
  };

  const displayLiftSquare = (data: any) => {
    if (data.liftState === false || data.liftState) {
      setLiftState(data.liftState);
      console.log(data.liftState);
    }
  };

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;
    displayRobot(data.update);
    displayPath(data.update);
    displayLiftSquare(data.update);
  };

  const stateCallback = (state: string) => {
    if (manager === null) {
      return;
    }

    if (state === states.TOOLS_READY) {
      const world = manager.getWorld();

      if (world.includes("2")) {
        setMap(Map2);
        setMapSize(Map2Size);
      } else {
        setMap(Map1);
        setMapSize(Map1Size);
      }

      if (world.includes("Ackermann")) {
        setVehicleType(1);
      } else {
        setVehicleType(0);
      }

      base_path = [];
      base_trail = [];
      setPath("");
      setTrail("");
      setVehiclePose(undefined);
      setTargetPose(undefined);
    }
  };

  connectApplication(
    manager,
    updateCallback,
    stateCallback,
    canvasRef,
    resizeObserver,
  );

  return (
    <WebGUIContainer>
      <WebGUIImage reference={canvasRef} src={map} style={{ width: "100%" }} />
      {vehiclePose && vehicleType == 0 && (
        <div
          id="vehic-pos"
          className={liftState ? "lifting" : ""}
          style={{
            rotate: "z " + (vehiclePose[2] + Math.PI / 2) + "rad",
            top: vehiclePose[0] - 10,
            left: vehiclePose[1] - 10,
          }}
        >
          <div className="arrow" />
        </div>
      )}
      {vehiclePose && vehicleType == 1 && (
        <div
          id="vehic-pos-ack"
          className={liftState ? "lifting-ack" : ""}
          style={{
            rotate: "z " + (vehiclePose[2] + Math.PI) + "rad",
            top: vehiclePose[0] - 25,
            left: vehiclePose[1] - 10,
          }}
        >
          <div className="arrow-ack arrow" />
        </div>
      )}
      {trail && (
        <svg
          height="100%"
          width="100%"
          xmlns="http://www.w3.org/2000/svg"
          style={{ zIndex: 3, position: "absolute" }}
        >
          <path
            xmlns="http://www.w3.org/2000/svg"
            d={trail}
            style={{
              strokeWidth: "2px",
              strokeLinejoin: "round",
              stroke: "blue",
              fill: "none",
            }}
          />
        </svg>
      )}
      {path && (
        <svg
          height="100%"
          width="100%"
          xmlns="http://www.w3.org/2000/svg"
          style={{ zIndex: 2, position: "absolute" }}
        >
          <path
            xmlns="http://www.w3.org/2000/svg"
            d={path}
            style={{
              strokeWidth: "2px",
              strokeLinejoin: "round",
              stroke: "green",
              fill: "none",
            }}
          />
        </svg>
      )}
      {targetPose && (
        <div
          className="target"
          style={{
            top: `${targetPose[0]}%`,
            left: `calc(${targetPose[1]}% - ${10}px)`,
          }}
        />
      )}
    </WebGUIContainer>
  );
}

export default WebGUI;
