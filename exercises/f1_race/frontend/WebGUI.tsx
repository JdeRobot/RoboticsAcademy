import React, { useState, useEffect } from "react";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";

import { contrastSelector } from "jderobot-ide-interface";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";

import F1Car from "./resources/f1-car.svg";
import Arrow from "./resources/arrow.svg";
import defaultCircuit from "./resources/default_circuit.png";
import "./css/GUICanvas.css";
import { getCarPose } from "./helpers/showCarPosition";

interface Race {
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

function WebGUI() {
  const theme = useAcademyTheme();
  const meter = 73; // 1m = 73px

  const [laser, setLaser] = useState<number[][]>([]);
  const [maxRange, setMaxRange] = useState<number>(1000);
  const [carForce, setCarForce] = useState<number[]>([2 * meter, 0]);
  const [avgForce, setAvgForce] = useState<number[]>([2 * meter, 0]);
  const [obsForce, setObsForce] = useState<number[]>([2 * meter, -Math.PI / 2]);
  const [targetPose, setTargetPose] = useState<number[] | undefined>(undefined);
  const [carPose, setCarPose] = useState<number[] | undefined>(undefined);
  const [rivalPose, setRivalPose] = useState<number[] | undefined>(undefined);
  const [race, setRace] = useState<Race | undefined>(undefined);
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const updateCallback = (updateData: unknown) => {
    let data = updateData as any;
    data = data.update;

    if (data.map) {
      const dataToDraw = JSON.parse(data.map);

      setLaser(dataToDraw.laser);
      setMaxRange(dataToDraw.max_range);
      const carForceDist = getDist(dataToDraw.car[0], dataToDraw.car[1]);
      setCarForce([
        carForceDist * meter,
        getAng(dataToDraw.car[0], dataToDraw.car[1]),
      ]);
      const avgForceDist = getDist(
        dataToDraw.average[0],
        dataToDraw.average[1],
      );
      setAvgForce([
        avgForceDist * meter,
        getAng(dataToDraw.average[0], dataToDraw.average[1]),
      ]);
      const obsForceDist = getDist(
        dataToDraw.obstacle[0],
        dataToDraw.obstacle[1],
      );
      setObsForce([
        obsForceDist * meter,
        getAng(dataToDraw.obstacle[0], dataToDraw.obstacle[1]),
      ]);
      const targetDist = getDist(
        dataToDraw.pose[0] - dataToDraw.target[0],
        dataToDraw.pose[1] - dataToDraw.target[1],
      );
      const targetAng = getAng(
        dataToDraw.pose[0] - dataToDraw.target[0],
        dataToDraw.pose[1] - dataToDraw.target[1],
      );
      setTargetPose([
        targetDist * meter,
        targetAng - dataToDraw.pose[2] - Math.PI,
      ]);
    }

    if (data.car) {
      setCarPose(getCarPose(data.car));
    }

    if (data.rival) {
      setRivalPose(getCarPose(data.rival));
    }

    if (data.race) {
      setRace(JSON.parse(data.race));
    }
  };

  const getDist = (x: number, y: number) => {
    return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  };

  const getAng = (x: number, y: number) => {
    const ang = Math.atan2(y, x);
    return ang;
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setTargetPose(undefined);
      setCarPose(undefined);
      setRivalPose(undefined);
      setRace(undefined);
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  const scoreBg = () => {
    if (race === undefined) {
      return undefined;
    }
    if (race.state === "won") {
      return theme.palette.success;
    }
    if (race.state === "lost") {
      return theme.palette.error;
    }
    return theme.palette.primary;
  };

  const scoreText = () => {
    return contrastSelector(
      theme.palette.text,
      theme.palette.darkText,
      scoreBg(),
    );
  };

  return (
    <WebGUIContainer id="f1-road">
      <img src={F1Car} id="f1-car" />
      {laser.map((element) => {
        const ang = -element[1];
        const length = (element[0] / maxRange) * 100;
        return (
          <hr
            className="laser-beam"
            style={{
              rotate: "z " + ang + "rad",
              width: "calc(" + length + "%)",
            }}
          />
        );
      })}
      <img
        className="arrow green"
        src={Arrow}
        style={{ height: carForce[0], rotate: "z " + -carForce[1] + "rad" }}
      />
      <img
        className="arrow red"
        src={Arrow}
        style={{ height: obsForce[0], rotate: "z " + -obsForce[1] + "rad" }}
      />
      <img
        className="arrow"
        src={Arrow}
        style={{
          height: avgForce[0],
          rotate: "z " + -avgForce[1] + "rad",
          zIndex: "6",
        }}
      />
      {targetPose && (
        <div
          className="target-container"
          style={{
            height: targetPose[0],
            rotate: "z " + -targetPose[1] + "rad",
          }}
        >
          <div id="target" />
        </div>
      )}
      {race && (
        <p
          style={{
            position: "absolute",
            top: 0,
            right: 0,
            padding: "10px",
            fontSize: "18px",
            color: scoreText(),
            background: scoreBg(),
            borderBottomLeftRadius: theme.viewRoundness,
            zIndex: 8,
            margin: 0,
            textAlign: "left",
          }}
        >
          {`${race?.elapsed.toFixed(1) || "0.0"} s`}
          <br />
          {`Gap: ${race?.gap.toFixed(1) || "0"} m`}
        </p>
      )}
      <div className="overlay" id="circuit-aerial">
        <img src={defaultCircuit} alt="" id="circuit-img" />
        {rivalPose && (
          <CarArrow
            id="race-rival-arrow"
            pose={rivalPose}
            fill="#00b4ff"
            stroke="#ffffff"
          />
        )}
        {carPose && (
          <CarArrow
            id="race-car-arrow"
            pose={carPose}
            fill="#ff8c00"
            stroke="#000000"
          />
        )}
      </div>
    </WebGUIContainer>
  );
}

export default WebGUI;
