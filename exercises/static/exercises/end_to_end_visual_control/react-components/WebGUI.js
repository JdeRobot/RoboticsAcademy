import * as React from "react";
import { useState, useEffect, useRef } from "react";
import { Box } from "@mui/material";
import "./css/GUICanvas.css";
import { getCarPose } from "./helpers/showCarPositionFollowLine";
import { displayLapTime } from "./helpers/showLapTimeFollowLine";
import { events, states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIContainer from "Components/exercise/WebGUIContainer";

import defaultCircuit from "../resources/images/default_circuit.png";
import montmeloCircuit from "../resources/images/montmelo_circuit.png";
import montrealCircuit from "../resources/images/montreal_circuit.png";
import ngbCircuit from "../resources/images/ngb_circuit.png";

const WebGUI = () => {
  const exerciseContext = useExercise();
  const canvasRef = useRef(null);
  const [lapTime, setLapTime] = useState(null);
  const [carPose, setCarPose] = useState(null);
  const [circuitImg, setCircuitImg] = useState(defaultCircuit);
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  var circuitName = "simple";

  useEffect(() => {
    if (manager) {
      updateCircuit(manager.getUniverse());
    }
  }, []);

  const updateCircuit = (universe) => {
    switch (universe) {
      case "Simple Circuit":
        circuitName = "default";
        setCircuitImg(defaultCircuit);
        break;
      case "Montmelo Circuit":
        circuitName = "montmelo";
        setCircuitImg(montmeloCircuit);
        break;
      case "Montreal Circuit":
        circuitName = "montreal";
        setCircuitImg(montrealCircuit);
        break;
      case "Nurburgring Circuit":
        circuitName = "ngb";
        setCircuitImg(ngbCircuit);
        break;
      case "Simple Ackermann Circuit":
        circuitName = "default ack";
        setCircuitImg(defaultCircuit);
        break;
      case "Montmelo Ackermann Circuit":
        circuitName = "montmelo ack";
        setCircuitImg(montmeloCircuit);
        break;
      case "Montreal Ackermann Circuit":
        circuitName = "montreal ack";
        setCircuitImg(montrealCircuit);
        break;
      case "Nurburgring Ackermann Circuit":
        circuitName = "ngb ack";
        setCircuitImg(ngbCircuit);
        break;
    }
  };

  useEffect(() => {
    if (manager === null) {
      return;
    }

    const updateCallback = (message) => {
      if (message.data.update.image) {
        const image = JSON.parse(message.data.update.image);
        if (image.image) {
          let canvas = document.getElementById("canvas");
          //Parse encoded image data and decode it
          function decode_utf8(s) {
            return decodeURIComponent(escape(s));
          }
          var source = decode_utf8(image.image),
            shape = image.shape;

          if (source !== "") {
            canvas.src = "data:image/png;base64," + source;
            canvas.width = shape[1];
            canvas.height = shape[0];
          }
        }
        try {
          const pose = getCarPose(circuitName, message.data.update.map);
          setLapTime(displayLapTime(message.data.update.lap));
          setCarPose(pose);
        } catch (error) {}
      }

      // Send the ACK of the msg
      manager.send("gui", "ack");
    };

    const stateCallback = (message) => {
      if (message.data.state === states.RUNNING) {
        manager.send("gui", "start");
      } else if (message.data.state === states.PAUSED) {
        manager.send("gui", "pause");
      } else if (message.data.state === states.TOOLS_READY) {
        setCarPose(null);
        setLapTime(null);
        updateCircuit(manager.getUniverse());
      }
    };

    manager.subscribe(events.UPDATE, updateCallback);
    manager.subscribe(events.STATE_CHANGED, stateCallback);

    return () => {
      manager.unsubscribe(events.UPDATE, updateCallback);
      manager.unsubscribe(events.STATE_CHANGED, stateCallback);
    };
  }, [manager]);

  return (
    <WebGUIContainer>
      <img ref={canvasRef} className={"exercise-canvas"} id="canvas"></img>
      {lapTime && (
        <label className="overlay" id="lap-time">
          {lapTime} s
        </label>
      )}
      <div className="overlay" id="circuit-aerial">
        <img src={circuitImg} alt="" id="circuit-img" />
        {carPose && (
          <div
            id="circuit-car-pos"
            style={{ top: carPose[1], left: carPose[0] }}
          />
        )}
      </div>
    </WebGUIContainer>
  );
};

export default WebGUI;
