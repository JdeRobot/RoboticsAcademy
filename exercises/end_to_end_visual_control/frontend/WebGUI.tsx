import React, { useState, useEffect } from "react";
import "./css/GUICanvas.css";
import { getCarPose } from "./helpers/showCarPositionFollowLine";
import { displayLapTime } from "./helpers/showLapTimeFollowLine";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import WebGUIImage from "Components/exercise/WebGUIImage";

import defaultCircuit from "./resources/default_circuit.png";
import montmeloCircuit from "./resources/montmelo_circuit.png";
import montrealCircuit from "./resources/montreal_circuit.png";
import ngbCircuit from "./resources/ngb_circuit.png";
import monacoCircuit from "./resources/monaco_circuit.png";

const WebGUI = () => {
  const exerciseContext = useExercise();
  const [image, setImage] = useState<string | undefined>(undefined);
  const [lapTime, setLapTime] = useState<string | undefined>(undefined);
  const [carPose, setCarPose] = useState<number[] | undefined>(undefined);
  const [circuitImg, setCircuitImg] = useState<string | undefined>(undefined);
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  let circuitName = "simple";

  useEffect(() => {
    if (manager) {
      updateCircuit(manager.getWorld());
    }
  }, []);

  const updateCircuit = (world: string) => {
    if (world === undefined) {
      return;
    }

    if (world.includes("Simple")) {
      circuitName = "default";
      setCircuitImg(defaultCircuit);
    } else if (world.includes("Montmelo")) {
      circuitName = "montmelo";
      setCircuitImg(montmeloCircuit);
    } else if (world.includes("Montreal")) {
      circuitName = "montreal";
      setCircuitImg(montrealCircuit);
    } else if (world.includes("Nurburgring")) {
      circuitName = "ngb";
      setCircuitImg(ngbCircuit);
    } else if (world.includes("Monaco")) {
      circuitName = "monaco";
      setCircuitImg(monacoCircuit);
    }
  };

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;
    if (data.update.image) {
      const image = JSON.parse(data.update.image);
      if (image.image != "" && image.shape instanceof Array) {
        setImage(`data:image/png;base64,${image.image}`);
      }
      try {
        const pose = getCarPose(circuitName, data.update.map);
        setCarPose(pose);
        setLapTime(displayLapTime(data.update.lap));
      } catch (error) {}
    }
  };

  const stateCallback = (state: string) => {
    if (manager === null) {
      return;
    }

    if (state === states.RUNNING) {
      manager.send("gui", "startLap");
    } else if (state === states.PAUSED) {
      manager.send("gui", "pause");
    } else if (state === states.TOOLS_READY) {
      setCarPose(undefined);
      setLapTime(undefined);
      setImage(undefined);
      updateCircuit(manager.getWorld());
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  return (
    <WebGUIContainer>
      <WebGUIImage style={{ width: "100%" }} src={image} />
      {lapTime && (
        <label className="overlay" id="lap-time">
          {lapTime} s
        </label>
      )}
      {circuitImg && (
        <div className="overlay" id="circuit-aerial">
          <img src={circuitImg} alt="" id="circuit-img" />
          {carPose && (
            <div
              id="circuit-car-pos"
              style={{ top: carPose[1], left: carPose[0] }}
            />
          )}
        </div>
      )}
    </WebGUIContainer>
  );
};

export default WebGUI;
