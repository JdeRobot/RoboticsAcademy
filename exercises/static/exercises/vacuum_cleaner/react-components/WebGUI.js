import { useState, useEffect, useRef } from "react";
import { updatePath, addToPath } from "./helpers/VacuumCleanerHelper";
import houseMapClean from "../resources/images/mapgrannyannie_clean.png";
import houseMapDirty from "../resources/images/mapgrannyannie_dirty.png";
import Vacuum from "../resources/images/vacuum.svg";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer from "Components/exercise/WebGUIContainer";
import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";

import "./css/GUICanvas.css";

const WebGUI = () => {
  const exerciseContext = useExercise();
  const [vacuumPose, setVacuumPose] = useState(null);
  const [path, setPath] = useState("");
  const [manager, setManager] = useState(exerciseContext.manager);
  var trail = [];
  var lastPose = undefined;
  const canvasRef = useRef(null);
  const vacuumSize = 40;

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const resizeObserver = new ResizeObserver((entries) => {
    // TODO: vacuum size = 30x30 en 750x750
    var img = entries[0].target;
    //or however you get a handle to the IMG
    var width = 1013 / 300 / (1013 / img.clientWidth);
    var height = 1012 / 150 / (1012 / img.clientHeight);

    updatePath(trail, setPath, height, width);
    if (lastPose) {
      setVacuumPose([lastPose[1] * height, lastPose[0] * width, -lastPose[2]]);
    }
  });

  useEffect(() => {
    if (manager === null) {
      return;
    }

    const updateCallback = (message) => {
      const updateData = message.data.update;
      // Lógica para manejar el mapa
      if (updateData.map) {
        const pose = updateData.map.substring(1, updateData.map.length - 1);
        const content = pose.split(",").map((item) => parseFloat(item));
        lastPose = content;

        var img = canvasRef.current;
        var width = img.clientWidth / 300;
        var height = img.clientHeight / 150;

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

      // Send the ACK of the msg
      manager.send("gui", "ack");
    };

    const stateCallback = (message) => {
      if (message.data.state === "tools_ready") {
        try {
          setPath("");
          trail = [];
        } catch (error) {}
      }
    };

    var img = canvasRef.current;
    if (img) {
      resizeObserver.observe(img);
    }

    manager.subscribe(events.UPDATE, updateCallback);
    manager.subscribe(events.STATE_CHANGED, stateCallback);

    return () => {
      manager.unsubscribe(events.UPDATE, updateCallback);
      manager.unsubscribe(events.STATE_CHANGED, stateCallback);
    };
  }, [manager]);

  return (
    <WebGUIContainer>
      <WebGUIImage
        reference={canvasRef}
        id="map-img"
        src={houseMapDirty}
        style={{ left: "0", width: "100%" }}
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
              <path xmlns="http://www.w3.org/2000/svg" d={path} style={{strokeWidth: "30px", strokeLinejoin:"round", stroke: "white", fill: "none"}}/>
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
    </WebGUIContainer>
  );
};

export default WebGUI;
