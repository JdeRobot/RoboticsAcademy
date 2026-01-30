import React, { useState, useEffect, useRef } from "react";
import { updatePath, addToPath } from "./helpers/VacuumCleanerHelper";
import houseMapDirty from "./resources/mapgrannyannie_dirty.png";
import Vacuum from "./resources/vacuum.svg";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";

import "./css/GUICanvas.css";

// Define multirun variables here so it does not rerender

const WebGUI = () => {
  const exerciseContext = useExercise();
  const [vacuumPose, setVacuumPose] = useState<number[] | undefined>(undefined);
  const [path, setPath] = useState<string>("");
  const [userImage, setUserImage] = useState<string | undefined>(undefined);
  const [manager, setManager] = useState(exerciseContext.manager);
  let lastPose: number[] | undefined = undefined;
  const canvasRef = useRef<HTMLImageElement>(null);
  const vacuumSize = 40;
  let trail: number[][] = [];

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const resizeObserver = new ResizeObserver((entries) => {
    const img = entries[0].target;
    const width = img.clientWidth / 300;
    const height = img.clientHeight / 150;

    updatePath(
      trail,
      setPath,
      height,
      (vacuumSize * img.clientHeight) / 1012,
      width,
      (vacuumSize * img.clientWidth) / 1012
    );

    if (lastPose) {
      setVacuumPose([
        lastPose[1] * height - (vacuumSize * img.clientHeight) / 1012,
        lastPose[0] * width - (vacuumSize * img.clientWidth) / 1012,
        -lastPose[2],
      ]);
    }
  });

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;
    const update = data.update;

    // Lógica para manejar el mapa
    if (update.map) {
      const pose = update.map.substring(1, update.map.length - 1);
      const content = pose.split(",").map((item: string) => parseFloat(item));
      lastPose = content;

      const img = canvasRef.current;
      if (img === null) {
        return;
      }
      const width = img.clientWidth / 300;
      const height = img.clientHeight / 150;

      updatePath(
        trail,
        setPath,
        height,
        (vacuumSize * img.clientHeight) / 1012,
        width,
        (vacuumSize * img.clientWidth) / 1012
      );

      setVacuumPose([
        content[1] * height - (vacuumSize * img.clientHeight) / 1012,
        content[0] * width - (vacuumSize * img.clientWidth) / 1012,
        -content[2],
      ]);
      addToPath(content[1], content[0], trail);
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
      setPath("");
      trail = [];
      setVacuumPose(undefined);
    }
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
        src={houseMapDirty}
        style={{ left: "0" }}
      />
      <div className="overlay" id="map-container">
        {vacuumPose && (
          <div
            id="vacuum-pos"
            style={{
              rotate: "z " + vacuumPose[2] + "rad",
              top: vacuumPose[0],
              height: (vacuumSize * canvasRef.current.clientHeight) / 1012,
              left: vacuumPose[1],
              width: (vacuumSize * canvasRef.current.clientWidth) / 1012,
            }}
          >
            <img src={Vacuum} />
            <div
              className="arrow"
              style={{
                height: (vacuumSize * canvasRef.current.clientHeight) / 1012,
                marginLeft:
                  -((vacuumSize * canvasRef.current.clientWidth) / 1012) / 2,
              }}
            />
          </div>
        )}
        {/* <svg height="100%" width="100%" xmlns="http://www.w3.org/2000/svg">
          <mask id="svg-draw" xmlns="http://www.w3.org/2000/svg">
            {path ? (
              <path xmlns="http://www.w3.org/2000/svg" d={path} style={{strokeWidth: "20px", strokeLinejoin:"round", stroke: "white", fill: "none"}}/>
            ) : (
              <path xmlns="http://www.w3.org/2000/svg"></path>
            )}
          </mask>
          <image href={houseMapClean} height="100%" width="100%" mask="url(#svg-draw)" preserveAspectRatio="none"></image>
        </svg> */}
        {path && (
          <svg
            height="100%"
            width="100%"
            xmlns="http://www.w3.org/2000/svg"
            style={{ zIndex: 2, position: "absolute", left: 0 }}
          >
            <path
              xmlns="http://www.w3.org/2000/svg"
              d={path}
              style={{
                strokeWidth:
                  (vacuumSize * canvasRef.current.clientHeight) / 1012,
                strokeLinejoin: "round",
                stroke: "white",
                fill: "none",
              }}
            />
          </svg>
        )}
      </div>
      <WebGUIImage src={userImage} id="map-img" style={{ left: "50%" }} />
    </WebGUIContainer>
  );
};

export default WebGUI;
