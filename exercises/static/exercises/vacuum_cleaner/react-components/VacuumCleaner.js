import { useState, useEffect} from "react";
import { updatePath, addToPath } from "./helpers/VacuumCleanerHelper";
import houseMapClean from "../resources/images/mapgrannyannie_clean.png";
import houseMapDirty from "../resources/images/mapgrannyannie_dirty.png";
import Vacuum from "../resources/images/vacuum.svg";

import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";

import "./css/GUICanvas.css";

const VacuumCleaner = () => {
  const exerciseContext = useExercise();
  const [vacuumPose, setVacuumPose] = useState(null);
  const [path, setPath] = useState("");
  const [manager, setManager] = useState(exerciseContext.manager);
  var trail = [];
  var lastPose = undefined;

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const resizeObserver = new ResizeObserver((entries) => {
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

        var img = document.getElementById("exercise-img");
        //or however you get a handle to the IMG
        var width = 1012 / 300 / (1012 / img.clientWidth);
        var height = 1012 / 150 / (1012 / img.clientHeight);

        updatePath(trail, setPath, height, width);

        setVacuumPose([content[1] * height, content[0] * width, -content[2]]);
        addToPath(content[1], content[0], trail);
      }

      // Send the ACK of the msg
      window.RoboticsExerciseComponents.commsManager.send("gui", "ack");
    };

    const stateCallback = (message) => {
      if (message.data.state === "tools_ready") {
        try {
          setPath("");
          trail = [];
        } catch (error) {}
      }
    };

    resizeObserver.observe(document.getElementById("exercise-img"));

    manager.subscribe(events.UPDATE, updateCallback);
    manager.subscribe(events.STATE_CHANGED, updateCallback);

    return () => {
      manager.unsubscribe(events.UPDATE, stateCallback);
      manager.unsubscribe(events.STATE_CHANGED, stateCallback);
    };
  }, [manager]);

  return (
    <div
      style={{
        display: "flex",
        width: "100%",
        height: "100%",
        position: "relative",
      }}
    >
      <img
        src={houseMapDirty}
        alt=""
        className="exercise-canvas"
        id="exercise-img"
      />
      <div className="overlay" id="map-container">
        {vacuumPose && (
          <div
            id="vacuum-pos"
            style={{
              rotate: "z " + vacuumPose[2] + "rad",
              top: vacuumPose[0] - 15,
              left: vacuumPose[1] - 15,
            }}
          >
            <img src={Vacuum} id="vacuum-pos" />
            <div className="arrow" />
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
            style={{ zIndex: 2, position: "absolute" }}
          >
            <path
              xmlns="http://www.w3.org/2000/svg"
              d={path}
              style={{
                strokeWidth: "30px",
                strokeLinejoin: "round",
                stroke: "white",
                fill: "none",
              }}
            />
          </svg>
        )}
      </div>
    </div>
  );
};

export default VacuumCleaner;
