import React, { useState, useEffect } from "react";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIContainer, { connectApplication } from "Components/exercise/WebGUIContainer";

import F1Car from "./resources/f1-car.svg";
import Arrow from "./resources/arrow.svg";
import "./css/GUICanvas.css";

function WebGUI() {
  const meter = 73; // 1m = 73px

  const [laser, setLaser] = useState<number[][]>([]);
  const [maxRange, setMaxRange] = useState<number>(1000);
  const [carForce, setCarForce] = useState<number[]>([2 * meter, 0]);
  const [avgForce, setAvgForce] = useState<number[]>([2 * meter, 0]);
  const [obsForce, setObsForce] = useState<number[]>([2 * meter, -Math.PI / 2]);
  const [targetPose, setTargetPose] = useState<number[] | undefined>(undefined);

  // 🔥 Velocidades actuales
  const [currentV, setCurrentV] = useState(0);
  const [currentW, setCurrentW] = useState(0);

  // 🔥 Ventana dinámica
  const [dynamicWindow, setDynamicWindow] = useState<[number, number, number][]>([]);

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
      setCarForce([carForceDist * meter, getAng(dataToDraw.car[0], dataToDraw.car[1])]);

      const avgForceDist = getDist(dataToDraw.average[0], dataToDraw.average[1]);
      setAvgForce([avgForceDist * meter, getAng(dataToDraw.average[0], dataToDraw.average[1])]);

      const obsForceDist = getDist(dataToDraw.obstacle[0], dataToDraw.obstacle[1]);
      setObsForce([obsForceDist * meter, getAng(dataToDraw.obstacle[0], dataToDraw.obstacle[1])]);

      const targetDist = getDist(
        dataToDraw.pose[0] - dataToDraw.target[0],
        dataToDraw.pose[1] - dataToDraw.target[1]
      );
      const targetAng = getAng(
        dataToDraw.pose[0] - dataToDraw.target[0],
        dataToDraw.pose[1] - dataToDraw.target[1]
      );
      setTargetPose([targetDist * meter, targetAng - dataToDraw.pose[2] - Math.PI]);

      // 🔥 Velocidad actual
      if (dataToDraw.bestVelocity) {
        setCurrentV(dataToDraw.bestVelocity[0]);
        setCurrentW(dataToDraw.bestVelocity[1]);
      }

      // 🔥 Ventana dinámica
      if (dataToDraw.dynamicWindow) {
        setDynamicWindow(
          dataToDraw.dynamicWindow.map((item: any) => [item[0], item[1], item[2]])
        );
      }
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

  // 🔥 Config visor velocidades
  const SPEED_SIZE = 200;
  const VMAX = 2.0;
  const WMAX = 2.0;

  const mapVW = (v: number, w: number) => {
    const x = SPEED_SIZE / 2 + (w / WMAX) * (SPEED_SIZE / 2 - 20);
    const y = SPEED_SIZE / 2 - (v / VMAX) * (SPEED_SIZE / 2 - 20);
    return [x, y];
  };

  // 🔥 Config heatmap ventana dinámica
  const HEAT_SIZE = 280;
  const HEAT_MARGIN = 10;

  return (
    <WebGUIContainer id="f1-road">
      {/* Coche y target */}
      <div style={{ position: "relative", width: "100%", height: "300px" }}>
        <img
          src={F1Car}
          id="f1-car"
          style={{ position: "absolute", left: "25%", top: "80%", transform: "translate(-50%, -50%)" }}
        />
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

        {laser.map((element, i) => {
          const ang = -element[1];
          const length = (element[0] / maxRange) * 100;

          return (
            <hr
              key={i}
              className="laser-beam"
              style={{
                position: "absolute",       // 🔥 clave
                left: "25%",                // 🔥 posición horizontal (ajústalo)
                top: "60%",                 // 🔥 altura (ajústalo)

                transformOrigin: "0% 50%",  // 🔥 rota desde el inicio del rayo
                transform: `rotate(${ang}rad)`,

                width: `${length}%`,
              }}
            />
          );
        })}
      </div>





      {/* 🔥 VISOR DE VELOCIDADES */}
      <div style={{ position: "absolute", left: "75%", top: "25%", transform: "translate(-50%, -50%)", width: "280px", height: "140px", zIndex: 10 }}>
        <svg width={280} height={140} style={{ border: "1.5px solid #333", borderRadius: "10px", background: "#fafafa", boxShadow: "0 4px 10px rgba(0,0,0,0.1)" }}>
          {[...Array(5)].map((_, i) => {
            const x = (i / 4) * 260;
            const y = (i / 4) * 140;
            return (
              <g key={i}>
                <line x1={x} y1={0} x2={x} y2={140} stroke="#ddd" strokeWidth="0.5" />
                <line x1={0} y1={y} x2={260} y2={y} stroke="#ddd" strokeWidth="0.5" />
              </g>
            );
          })}
          <line x1={130} y1={0} x2={130} y2={140} stroke="#444" strokeWidth="1.5" />
          <line x1={0} y1={70} x2={280} y2={70} stroke="#444" strokeWidth="1.5" />
          <text x={240} y={65} fontSize="12" fill="#333">w-</text>
          <text x={125} y={15} fontSize="12" fill="#333">v+</text>
          {(() => {
            const x = 130 - (currentW / WMAX) * 130;
            const y = 70 - (currentV / VMAX) * 70;
            return <>
              <circle cx={x} cy={y} r={7} fill="#ff4d4d" stroke="#222" strokeWidth={1.5} />
              <circle cx={x} cy={y} r={12} fill="none" stroke="#ff4d4d" opacity={0.3} />
            </>;
          })()}
          <text x={10} y={110} fontSize="13" fill="#222">v: {currentV.toFixed(2)} m/s</text>
          <text x={10} y={130} fontSize="13" fill="#222">w: {currentW.toFixed(2)} rad/s</text>
        </svg>
      </div>

      {/* 🔥 HEATMAP VENTANA DINÁMICA */}
      <div
        style={{
          position: "absolute",
          left: "75%",          // misma columna que el visor de velocidades
          top: "50%",           // un poco más abajo que el visor
          transform: "translate(-50%, 0)", 
          width: `${HEAT_SIZE}px`,
          height: `${HEAT_SIZE / 2}px`,   // tamaño real del contenedor
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
          {/* Fondo grid */}
          {[...Array(5)].map((_, i) => {
            const w = HEAT_SIZE;
            const h = HEAT_SIZE / 2;
            const posX = HEAT_MARGIN + (i / 4) * (w - 2 * HEAT_MARGIN);
            const posY = HEAT_MARGIN + (i / 4) * (h - 2 * HEAT_MARGIN);
            return (
              <g key={i}>
                <line x1={HEAT_MARGIN} y1={posY} x2={w - HEAT_MARGIN} y2={posY} stroke="#ddd" strokeWidth="0.5" />
                <line x1={posX} y1={HEAT_MARGIN} x2={posX} y2={h - HEAT_MARGIN} stroke="#ddd" strokeWidth="0.5" />
              </g>
            );
          })}

          {/* Escalas dinámicas con w negativa a la izquierda, v arriba */}
          {dynamicWindow.length > 0 && (() => {
            const wContainer = HEAT_SIZE;
            const hContainer = HEAT_SIZE / 2;

            const vValues = dynamicWindow.map(d => d[0]);
            const wValues = dynamicWindow.map(d => d[1]);
            const minV = Math.min(...vValues);
            const maxV = Math.max(...vValues);
            const minW = Math.min(...wValues);
            const maxW = Math.max(...wValues);

            return dynamicWindow.map(([v, w, score], i) => {
              const x = HEAT_MARGIN + ((maxW - w) / (maxW - minW)) * (wContainer - 2 * HEAT_MARGIN);
              const y = hContainer - HEAT_MARGIN - ((v - minV) / (maxV - minV)) * (hContainer - 2 * HEAT_MARGIN);
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
            dynamicWindow.length > 0 && (() => {
              const wContainer = HEAT_SIZE;
              const hContainer = HEAT_SIZE / 2;

              const vValues = dynamicWindow.map(d => d[0]);
              const wValues = dynamicWindow.map(d => d[1]);
              const minV = Math.min(...vValues);
              const maxV = Math.max(...vValues);
              const minW = Math.min(...wValues);
              const maxW = Math.max(...wValues);

              const x = HEAT_MARGIN + ((maxW - currentW) / (maxW - minW)) * (wContainer - 2 * HEAT_MARGIN);
              const y = hContainer - HEAT_MARGIN - ((currentV - minV) / (maxV - minV)) * (hContainer - 2 * HEAT_MARGIN);

              return <circle cx={x} cy={y} r={8} fill="none" stroke="black" strokeWidth={2} />;
            })()}
        </svg>
      </div>
    </WebGUIContainer>
  );
}

export default WebGUI;