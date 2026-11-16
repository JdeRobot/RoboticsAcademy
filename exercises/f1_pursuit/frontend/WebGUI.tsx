import React, { useState, useEffect, useRef } from "react";
import "./css/GUICanvas.css";
import { getCarPose } from "./helpers/showCarPosition";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import WebGUIImage from "Components/exercise/WebGUIImage";

import defaultCircuit from "./resources/default_circuit.png";
import montmeloCircuit from "./resources/montmelo_circuit.png";
import montrealCircuit from "./resources/montreal_circuit.png";

interface Pursuit {
  state: string;
  level: string;
  elapsed: number;
  limit: number;
  gap: number;
  score: number;
}

// Points right towards the car heading, colors inline so no stale stylesheet can hide them
const CarArrow = ({
  id,
  pose,
  fill,
  stroke,
}: {
  id: string;
  pose: number[];
  fill: string;
  stroke: string;
}) => (
  <svg
    id={id}
    className="circuit-arrow"
    width="16"
    height="14"
    viewBox="0 0 16 14"
    style={{
      position: "absolute",
      overflow: "visible",
      top: pose[1],
      left: pose[0],
      transform: `translate(-50%, -50%) rotate(${pose[2]}deg)`,
    }}
  >
    <polygon
      points="1,1 15,7 1,13 5,7"
      fill={fill}
      stroke={stroke}
      strokeWidth="1.5"
      strokeLinejoin="round"
    />
  </svg>
);

const WebGUI = () => {
  const exerciseContext = useExercise();
  const [image, setImage] = useState<string | undefined>(undefined);
  const [carPose, setCarPose] = useState<number[] | undefined>(undefined);
  const [rivalPose, setRivalPose] = useState<number[] | undefined>(undefined);
  const [pursuit, setPursuit] = useState<Pursuit | undefined>(undefined);
  const [circuit, setCircuit] = useState<string>("default");
  // The update callback is subscribed once, a ref keeps it reading the current circuit
  const circuitRef = useRef<string>("default");
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  useEffect(() => {
    if (manager) {
      updateCircuit(manager.getWorld());
    }
  }, []);

  // The world name tells the circuit, like follow_line
  const updateCircuit = (world: string) => {
    if (world === undefined) {
      return;
    }
    let name = "default";
    if (world.includes("Montmelo")) {
      name = "montmelo";
    } else if (world.includes("Montreal")) {
      name = "montreal";
    }
    circuitRef.current = name;
    setCircuit(name);
  };

  const circuitImg = {
    default: defaultCircuit,
    montmelo: montmeloCircuit,
    montreal: montrealCircuit,
  }[circuit];

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;
    if (data.update.image) {
      const image = JSON.parse(data.update.image);
      if (image.image != "" && image.shape instanceof Array) {
        setImage(`data:image/png;base64,${image.image}`);
      }
      try {
        setCarPose(getCarPose(circuitRef.current, data.update.map));
        setRivalPose(getCarPose(circuitRef.current, data.update.rival));
      } catch (error) {}
      try {
        if (data.update.pursuit) {
          setPursuit(JSON.parse(data.update.pursuit));
        }
      } catch (error) {}
    }
  };

  const stateCallback = (state: string) => {
    if (manager === null) {
      return;
    }
    if (state === states.TOOLS_READY) {
      setCarPose(undefined);
      setRivalPose(undefined);
      setPursuit(undefined);
      setImage(undefined);
      updateCircuit(manager.getWorld());
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  const scoreboard = () => {
    if (pursuit === undefined) {
      return undefined;
    }
    if (pursuit.state === "caught") {
      return `CAUGHT in ${pursuit.elapsed.toFixed(1)} s | score ${pursuit.score}`;
    }
    if (pursuit.state === "lapped") {
      return `LAPPED, the rival caught you | score ${pursuit.score}`;
    }
    return `${pursuit.gap.toFixed(1)} m behind | ${pursuit.elapsed.toFixed(
      1,
    )} / ${pursuit.limit.toFixed(0)} s | score ${pursuit.score}`;
  };

  // Green once caught, red when lapped or past the par time
  const scoreColor = () => {
    if (pursuit === undefined) {
      return undefined;
    }
    if (pursuit.state === "caught") {
      return "#2ecc40";
    }
    if (pursuit.state === "lapped" || pursuit.score < 0) {
      return "red";
    }
    return "#ff9800";
  };

  return (
    <WebGUIContainer>
      <WebGUIImage style={{ width: "100%" }} src={image} />
      {pursuit && (
        <label
          className="overlay"
          id="pursuit-status"
          style={{ color: scoreColor() }}
        >
          {scoreboard()}
        </label>
      )}
      <div className="overlay" id="circuit-aerial">
        <img src={circuitImg} alt="" id="circuit-img" />
        {rivalPose && (
          <CarArrow
            id="pursuit-rival-arrow"
            pose={rivalPose}
            fill="#00b4ff"
            stroke="#ffffff"
          />
        )}
        {carPose && (
          <CarArrow
            id="pursuit-car-arrow"
            pose={carPose}
            fill="#ff8c00"
            stroke="#000000"
          />
        )}
      </div>
    </WebGUIContainer>
  );
};

export default WebGUI;
