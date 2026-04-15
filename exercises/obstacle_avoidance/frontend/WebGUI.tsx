import React, { useState, useEffect } from "react";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";

import F1Car from "./resources/f1-car.svg";
import Arrow from "./resources/arrow.svg";
import "./css/GUICanvas.css";

function WebGUI() {
  const meter = 73;

  const VMAX = 2.0;
  const WMAX = 2.0;
  const VIEW_MARGIN = 10;
  const HEAT_SIZE = 280;
  const HEAT_MARGIN = 10;

  const [laser, setLaser] = useState<number[][]>([]);
  const [maxRange, setMaxRange] = useState<number>(1000);

  const [carForce, setCarForce] = useState<number[]>([2 * meter, 0]);
  const [avgForce, setAvgForce] = useState<number[]>([2 * meter, 0]);
  const [obsForce, setObsForce] = useState<number[]>([2 * meter, -Math.PI / 2]);

  const [currentV, setCurrentV] = useState(0);
  const [currentW, setCurrentW] = useState(0);

  const [targetPose, setTargetPose] = useState<number[] | undefined>(undefined);

  const [mode, setMode] = useState<"forces" | "window" | null>(null);
  const [dynamicWindow, setDynamicWindow] = useState<any[]>([]);

  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const updateCallback = (updateData: unknown) => {
    let data = updateData as any;
    data = data.update;

    if (!data.map) return;

    const dataToDraw = JSON.parse(data.map);

    setLaser(dataToDraw.laser);
    setMaxRange(dataToDraw.max_range);

    // ---------------- DECIDE MODE ONCE ----------------
    if (mode === null) {
      if (dataToDraw.dynamicWindow) {
        setMode("window");
      } else {
        setMode("forces");
      }
    }

    // ---------------- FORCES DATA ----------------
    if (dataToDraw.car) {
      const d = getDist(dataToDraw.car[0], dataToDraw.car[1]);
      setCarForce([d * meter, getAng(dataToDraw.car[0], dataToDraw.car[1])]);
    }

    if (dataToDraw.average) {
      const d = getDist(dataToDraw.average[0], dataToDraw.average[1]);
      setAvgForce([d * meter, getAng(dataToDraw.average[0], dataToDraw.average[1])]);
    }

    if (dataToDraw.obstacle) {
      const d = getDist(dataToDraw.obstacle[0], dataToDraw.obstacle[1]);
      setObsForce([d * meter, getAng(dataToDraw.obstacle[0], dataToDraw.obstacle[1])]);
    }

    if (dataToDraw.bestVelocity) {
      setCurrentV(dataToDraw.bestVelocity[0]);
      setCurrentW(dataToDraw.bestVelocity[1]);
    }

    // ---------------- WINDOW DATA ----------------
    if (dataToDraw.dynamicWindow) {
      setDynamicWindow(
        dataToDraw.dynamicWindow.map((i: any) => [i[0], i[1], i[2]])
      );
    }

    // ---------------- TARGET ----------------
    if (dataToDraw.pose && dataToDraw.target) {
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

  const getDist = (x: number, y: number) => Math.sqrt(x * x + y * y);
  const getAng = (x: number, y: number) => Math.atan2(y, x);

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setTargetPose(undefined);
    }
  };

  connectApplication(manager, updateCallback);

  // =========================================================
  // ===================== FORCES SCENE ======================
  // =========================================================
  const ForcesScene = () => (
    <div style={{ position: "relative", width: "100%", height: "300px" }}>
      <img src={F1Car} id="f1-car" />

      {laser.map((element, i) => {
        const ang = -element[1];
        const length = (element[0] / maxRange) * 100;

        return (
          <hr
            key={i}
            className="laser-beam"
            style={{
              rotate: "z " + ang + "rad",
              width: `${length}%`,
            }}
          />
        );
      })}

      <img className="arrow green" src={Arrow}
        style={{ height: carForce[0], rotate: "z " + -carForce[1] + "rad" }}
      />

      <img className="arrow red" src={Arrow}
        style={{ height: obsForce[0], rotate: "z " + -obsForce[1] + "rad" }}
      />

      <img className="arrow" src={Arrow}
        style={{
          height: avgForce[0],
          rotate: "z " + -avgForce[1] + "rad",
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
    </div>
  );

  // =========================================================
  // ===================== WINDOW SCENE ======================
  // =========================================================
  const WindowScene = () => (
    <div style={{ position: "relative", width: "100%", height: "300px" }}>
      {/* ---------------- COCHE ---------------- */}
      <img
        src={F1Car}
        id="f1-car"
        style={{
          position: "absolute",
          left: "25%",
          top: "80%",
          transform: "translate(-50%, -50%)",
        }}
      />

      {/* ---------------- TARGET ---------------- */}
      {targetPose && (
        <div
          className="target-container"
          style={{
            position: "absolute",
            left: "25%",
            top: "80%",
            transform: `translate(-50%, -50%) rotate(${-targetPose[1]}rad)`,
            height: targetPose[0],
          }}
        >
          <div id="target" />
        </div>
      )}

      {/* ---------------- LASER ---------------- */}
      {laser.map((element, i) => {
        const ang = -element[1];
        const length = (element[0] / maxRange) * 100;

        return (
          <hr
            key={i}
            className="laser-beam"
            style={{
              position: "absolute",
              left: "25%",
              top: "60%",
              transformOrigin: "0% 50%",
              transform: `rotate(${ang}rad)`,
              width: `${length}%`,
            }}
          />
        );
      })}

      {/* ---------------- VISOR DE VELOCIDADES ---------------- */}
      <div
        style={{
          position: "absolute",
          left: "75%",
          top: "25%",
          transform: "translate(-50%, -50%)",
          width: "280px",
          height: "140px",
          zIndex: 10,
        }}
      >
        <svg
          width={280}
          height={140}
          style={{
            border: "1.5px solid #333",
            borderRadius: "10px",
            background: "#fafafa",
            boxShadow: "0 4px 10px rgba(0,0,0,0.1)",
          }}
        >
          {(() => {
            const lines = [];

            // 🔹 verticales (w)
            for (let w = -WMAX; w <= WMAX; w += 0.5) {
              const x = 130 - (w / WMAX) * (130 - VIEW_MARGIN);

              lines.push(
                <line
                  key={`w-${w}`}
                  x1={x}
                  y1={VIEW_MARGIN}
                  x2={x}
                  y2={140 - VIEW_MARGIN}
                  stroke={w === 0 ? "#444" : "#ddd"}
                  strokeWidth={w === 0 ? 1.5 : 0.5}
                />
              );
            }

            // 🔹 horizontales (v)
            for (let v = 0; v <= VMAX; v += 0.5) {
              const y = 70 - (v / VMAX) * (70 - VIEW_MARGIN);

              lines.push(
                <line
                  key={`v-${v}`}
                  x1={VIEW_MARGIN}
                  y1={y}
                  x2={280 - VIEW_MARGIN}
                  y2={y}
                  stroke={v === 0 ? "#444" : "#ddd"}
                  strokeWidth={v === 0 ? 1.5 : 0.5}
                />
              );
            }

            return lines;
          })()}

          {/* ejes */}
          <line
            x1={130}
            y1={VIEW_MARGIN}
            x2={130}
            y2={140 - VIEW_MARGIN}
            stroke="#444"
            strokeWidth="1.5"
          />
          <line
            x1={VIEW_MARGIN}
            y1={70}
            x2={280 - VIEW_MARGIN}
            y2={70}
            stroke="#444"
            strokeWidth="1.5"
          />

          <text x={240} y={65} fontSize="12" fill="#333">
            w-
          </text>
          <text x={125} y={15} fontSize="12" fill="#333">
            v+
          </text>

          {/* punto actual */}
          <circle
            cx={130 - (currentW / WMAX) * (130 - VIEW_MARGIN)}
            cy={70 - (currentV / VMAX) * (70 - VIEW_MARGIN)}
            r={5}
            fill="#ff4d4d"
            stroke="#222"
            strokeWidth={1.5}
          />

          {/* rango dinámico */}
          {(() => {
            const maxAccV = 2;
            const maxAccW = 2;
            const dt = 0.1;

            const vMin = Math.max(0, currentV - maxAccV * dt);
            const vMax = Math.min(VMAX, currentV + maxAccV * dt);

            const wMin = Math.max(-WMAX, currentW - maxAccW * dt);
            const wMax = Math.min(WMAX, currentW + maxAccW * dt);

            const x1 = 130 - (wMax / WMAX) * (130 - VIEW_MARGIN);
            const x2 = 130 - (wMin / WMAX) * (130 - VIEW_MARGIN);
            const y1 = 70 - (vMax / VMAX) * (70 - VIEW_MARGIN);
            const y2 = 70 - (vMin / VMAX) * (70 - VIEW_MARGIN);

            return (
              <rect
                x={Math.min(x1, x2)}
                y={Math.min(y1, y2)}
                width={Math.abs(x2 - x1)}
                height={Math.abs(y2 - y1)}
                fill="none"
                stroke="black"
                strokeWidth={2}
              />
            );
          })()}

          <text x={10} y={110} fontSize="13" fill="#222">
            v: {currentV.toFixed(2)} m/s
          </text>
          <text x={10} y={130} fontSize="13" fill="#222">
            w: {currentW.toFixed(2)} rad/s
          </text>
        </svg>
      </div>

      {/* ---------------- HEATMAP VENTANA DINÁMICA ---------------- */}
      <div
        style={{
          position: "absolute",
          left: "75%",
          top: "50%",
          transform: "translate(-50%, 0)",
          width: `${HEAT_SIZE}px`,
          height: `${HEAT_SIZE / 2}px`,
          zIndex: 10,
        }}
      >
        <svg
          width={HEAT_SIZE}
          height={HEAT_SIZE / 2}
          style={{
            border: "1.5px solid #333",
            borderRadius: "10px",
            background: "#fafafa",
            boxShadow: "0 4px 10px rgba(0,0,0,0.1)",
          }}
        >
          {dynamicWindow.length > 0 && (() => {
            const wContainer = HEAT_SIZE;
            const hContainer = HEAT_SIZE / 2;

            const vValues = dynamicWindow.map(d => d[0]);
            const wValues = dynamicWindow.map(d => d[1]);

            const minV = Math.min(...vValues);
            const maxV = Math.max(...vValues);
            const minW = Math.min(...wValues);
            const maxW = Math.max(...wValues);

            const safeW = (maxW - minW) || 1;
            const safeV = (maxV - minV) || 1;

            return dynamicWindow.map(([v, w, score], i) => {
              const x =
                HEAT_MARGIN +
                ((maxW - w) / safeW) * (wContainer - 2 * HEAT_MARGIN);

              const y =
                hContainer -
                HEAT_MARGIN -
                ((v - minV) / safeV) * (hContainer - 2 * HEAT_MARGIN);

              const r = Math.floor(255 * (1 - score));
              const g = Math.floor(255 * score);

              return (
                <circle
                  key={i}
                  cx={x}
                  cy={y}
                  r={4}
                  fill={`rgb(${r},${g},0)`}
                />
              );
            });
          })()}

          {/* Mejor velocidad */}
          {currentV !== undefined &&
            currentW !== undefined &&
            dynamicWindow.length > 0 &&
            (() => {
              const wContainer = HEAT_SIZE;
              const hContainer = HEAT_SIZE / 2;

              const vValues = dynamicWindow.map(d => d[0]);
              const wValues = dynamicWindow.map(d => d[1]);

              const minV = Math.min(...vValues);
              const maxV = Math.max(...vValues);
              const minW = Math.min(...wValues);
              const maxW = Math.max(...wValues);

              const safeW = (maxW - minW) || 1;
              const safeV = (maxV - minV) || 1;

              const x =
                HEAT_MARGIN +
                ((maxW - currentW) / safeW) * (wContainer - 2 * HEAT_MARGIN);

              const y =
                hContainer -
                HEAT_MARGIN -
                ((currentV - minV) / safeV) * (hContainer - 2 * HEAT_MARGIN);

              return (
                <circle
                  cx={x}
                  cy={y}
                  r={8}
                  fill="none"
                  stroke="black"
                  strokeWidth={2}
                />
              );
            })()}
        </svg>
      </div>
    </div>
  );

  return (
    <WebGUIContainer id="f1-road">
      {mode === "forces" && <ForcesScene />}
      {mode === "window" && <WindowScene />}
    </WebGUIContainer>
  );
}

export default WebGUI;