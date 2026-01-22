import * as React from "react";
import { useState, useEffect } from "react";
import "./css/GUICanvas.css";
import { getCarPose } from "./helpers/showCarPositionFollowLine";
import { displayLapTime } from "./helpers/showLapTimeFollowLine";
import { events, states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import WebGUIImage from "Components/exercise/WebGUIImage";

import defaultCircuit from "../resources/images/default_circuit.png";
import montmeloCircuit from "../resources/images/montmelo_circuit.png";
import montrealCircuit from "../resources/images/montreal_circuit.png";
import ngbCircuit from "../resources/images/ngb_circuit.png";
import monacoCircuit from "../resources/images/monaco_circuit.png";

const WebGUI = () => {
  const exerciseContext = useExercise();
  const [image, setImage] = useState();
  const [lapTime, setLapTime] = useState(null);
  const [carPose, setCarPose] = useState(null);
  const [circuitImg, setCircuitImg] = useState(defaultCircuit);
  const [manager, setManager] = useState(exerciseContext.manager);
  let connection = connectApplication(manager);

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
    if (universe === undefined) {
      return;
    }

    if (universe.includes("Simple")) {
      circuitName = "default";
      setCircuitImg(defaultCircuit);
    } else if (universe.includes("Montmelo")) {
      circuitName = "montmelo";
      setCircuitImg(montmeloCircuit);
    } else if (universe.includes("Montreal")) {
      circuitName = "montreal";
      setCircuitImg(montrealCircuit);
    } else if (universe.includes("Nurburgring")) {
      circuitName = "ngb";
      setCircuitImg(ngbCircuit);
    } else if (universe.includes("Monaco")) {
      circuitName = "monaco";
      setCircuitImg(monacoCircuit);
    }
  };

  useEffect(() => {
    if (manager === null) {
      return;
    }

    

    const updateCallback = (message) => {
      connection.end();
      if (message.data.update.image) {
        const image = JSON.parse(message.data.update.image);
        if (image.image != "" && image.shape instanceof Array) {
          setImage(`data:image/png;base64,${image.image}`);
        }
        try {
          const pose = getCarPose(circuitName, message.data.update.map);
          setCarPose(pose);
          setLapTime(displayLapTime(message.data.update.lap));
        } catch (error) {}
      }

      // Send the ACK of the msg
      manager.send("gui", "ack");
    };

    const stateCallback = (message) => {
      if (message.data.state === states.RUNNING) {
        manager.send("gui", "startLap");
      } else if (message.data.state === states.PAUSED) {
        manager.send("gui", "pause");
      } else if (message.data.state === states.TOOLS_READY) {
        setCarPose(null);
        setLapTime(null);
        setImage();
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
      <WebGUIImage style={{ width: "100%" }} src={image} />
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
