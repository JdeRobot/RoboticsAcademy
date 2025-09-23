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
  const [userImage, setUserImage] = useState(undefined);
  const canvasRef = useRef(null);
  const [manager, setManager] = useState(exerciseContext.manager);
  var trail = [];
  var lastPose = undefined;

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const resizeObserver = new ResizeObserver((entries) => {
    var img = entries[0].target;
    //or however you get a handle to the IMG
    var width = 1012 / 300 / (1012 / img.clientWidth);
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

        var img = canvasRef.current
        //or however you get a handle to the IMG
        var width = 1013 / 300 / (1013 / img.clientWidth);
        var height = 1012 / 150 / (1012 / img.clientHeight);

        updatePath(trail, setPath, height, width);

        setVacuumPose([content[1] * height, content[0] * width, -content[2]]);
        addToPath(content[1], content[0], trail);
      }

      if (updateData.image) {
        let image = JSON.parse(updateData.image);
        if (image.shape instanceof Array) {
          setUserImage(`data:image/png;base64,${image.image}`);
        }
      }

      // Send the ACK of the msg
      manager.send("gui", "ack");
    };

    const stateCallback = (message) => {
      if (message.data.state === "tools_ready") {
        setPath("");
        setUserImage(undefined);
        trail = [];
      }
    };

    var img = canvasRef.current
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
      <WebGUIImage reference={canvasRef} id="map-img" src={houseMapDirty} style={{ left: "0" }} />
      <div className="overlay" id="map-container">
        {vacuumPose && (
          <div
            id="vacuum-pos"
            style={{
              rotate: "z " + vacuumPose[2] + "rad",
              top: vacuumPose[0] - 10,
              left: vacuumPose[1] - 10,
            }}
          >
            <img src={Vacuum} id="vacuum-pos" />
            <div className="arrow" />
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
                strokeWidth: "20px",
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
