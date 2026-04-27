import React, { useState, useEffect } from "react";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIContainer, { connectApplication } from "Components/exercise/WebGUIContainer";
import F1Car from "./resources/f1-car.svg";
import Arrow from "./resources/arrow.svg";
import "./css/GUICanvas.css";
import { displayLapTime } from "./helpers/showLapTimeFollowLine";

function WebGUI() {
  const meter = 73;

  const [laser, setLaser] = useState<number[][]>([]);
  const [maxRange, setMaxRange] = useState<number>(1000);
  const [carForce, setCarForce] = useState<number[]>([2 * meter, 0]);
  const [avgForce, setAvgForce] = useState<number[]>([2 * meter, 0]);
  const [obsForce, setObsForce] = useState<number[]>([2 * meter, -Math.PI / 2]);
  const [targetPose, setTargetPose] = useState<number[] | undefined>(undefined);
  const [lapTime, setLapTime] = useState<string | undefined>(undefined);

  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const updateCallback = (updateData: unknown) => {
    let data = updateData as any;
    data = data.update;

    if (data.lap !== undefined && data.lap !== "") {
      try {
        setLapTime(displayLapTime(data.lap));
      } catch (error) {}
    }

    if (data.map) {
      const dataToDraw = JSON.parse(data.map);

      setLaser(dataToDraw.laser);
      setMaxRange(dataToDraw.max_range);
      const carForceDist = getDist(dataToDraw.car[0], dataToDraw.car[1]);
      setCarForce([
        carForceDist * meter,
        getAng(dataToDraw.car[0], dataToDraw.car[1]),
      ]);
      const avgForceDist = getDist(dataToDraw.average[0], dataToDraw.average[1]);
      setAvgForce([
        avgForceDist * meter,
        getAng(dataToDraw.average[0], dataToDraw.average[1]),
      ]);
      const obsForceDist = getDist(dataToDraw.obstacle[0], dataToDraw.obstacle[1]);
      setObsForce([
        obsForceDist * meter,
        getAng(dataToDraw.obstacle[0], dataToDraw.obstacle[1]),
      ]);
      const targetDist = getDist(
        dataToDraw.pose[0] - dataToDraw.target[0],
        dataToDraw.pose[1] - dataToDraw.target[1]
      );
      const targetAng = getAng(
        dataToDraw.pose[0] - dataToDraw.target[0],
        dataToDraw.pose[1] - dataToDraw.target[1]
      );
      setTargetPose([
        targetDist * meter,
        targetAng - dataToDraw.pose[2] - Math.PI,
      ]);
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
    if (manager === null) {
      return;
    }

    if (state === states.RUNNING) {
      manager.send("gui", "startLap");
    } else if (state === states.PAUSED) {
      manager.send("gui", "pause");
    } else if (state === states.TOOLS_READY) {
      setTargetPose(undefined);
      setLapTime(undefined);
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  return (
    <WebGUIContainer id="f1-road">
      {lapTime && (
        <label className="overlay" id="lap-time" style={{ position: "absolute", top: "10px", left: "10px", zIndex: 100, backgroundColor: "rgba(0,0,0,0.5)", color: "white", padding: "5px", borderRadius: "5px", fontSize: "20px" }}>
          {lapTime} s
        </label>
      )}

      <img src={F1Car} id="f1-car" />
      {laser.map((element, index) => {
        const ang = -element[1];
        const length = (element[0] / maxRange) * 100;
        return (
          <hr
            key={index}
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
    </WebGUIContainer>
  );
}

export default WebGUI;