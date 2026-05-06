import React, { useState, useEffect, useRef } from "react";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import { useExercise } from "Contexts/ExerciseContext";
import { states } from "jderobot-commsmanager";

function WebGUI() {
  const exerciseContext = useExercise();

  const [image, setImage] = useState<string | undefined>(undefined);

  const [trajectory, setTrajectory] = useState<{ x: number; y: number }[]>([]);
  const [pos, setPos] = useState({ x: 0, y: 0 });
  const [drift, setDrift] = useState(0);

  const canvasRef = useRef<HTMLCanvasElement>(null);

  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;
    const update = data.update;

    if (update.image) {
      try {
        const parsed =
          typeof update.image === "string"
            ? JSON.parse(update.image)
            : update.image;

        const base64 = parsed?.image || parsed || update.image;

        if (base64) {
          setImage(`data:image/jpeg;base64,${base64}`);
        }
      } catch {
        setImage(`data:image/jpeg;base64,${update.image}`);
      }
    }

    if (update.odom) {
      const odomData = JSON.parse(update.odom);

      const scale = 1.2;

      const newPos = {
        x: odomData.x * scale,
        y: odomData.y * scale,
      };

      setPos(newPos);
      setDrift(odomData.drift ?? 0);

      setTrajectory((prev) => [...prev, newPos].slice(-1500));
    }
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setImage(undefined);
      setTrajectory([]);
      setPos({ x: 0, y: 0 });
      setDrift(0);
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  // =========================================================
  // ODOMETRY DRAW (FIXED RESPONSIVE + BOUNDS)
  // =========================================================
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;

    ctx.clearRect(0, 0, w, h);

    const cx = w / 2;
    const cy = h / 2;

    // GRID
    ctx.fillStyle = "#444";
    ctx.fillRect(cx, 0, 1, h);
    ctx.fillRect(0, cy, w, 1);

    // 🔥 LIMITE DEL MUNDO (evita que se salga)
    const WORLD_LIMIT = 80;

    // 🔥 escala automática según canvas
    const scale = Math.min(w, h) / (2 * WORLD_LIMIT);

    const clamp = (v: number, min: number, max: number) =>
      Math.max(min, Math.min(max, v));

    const worldToScreen = (p: { x: number; y: number }) => {
      const x = cx + clamp(p.y, -WORLD_LIMIT, WORLD_LIMIT) * scale;
      const y = cy - clamp(p.x, -WORLD_LIMIT, WORLD_LIMIT) * scale;
      return { x, y };
    };

    // TRAJECTORY (RED LINE)
    ctx.strokeStyle = "red";
    ctx.lineWidth = 2;
    ctx.beginPath();

    trajectory.forEach((p, i) => {
      const s = worldToScreen(p);

      if (i === 0) ctx.moveTo(s.x, s.y);
      else ctx.lineTo(s.x, s.y);
    });

    ctx.stroke();

    // CURRENT POSITION (GREEN)
    const s = worldToScreen(pos);

    ctx.fillStyle = "lime";
    ctx.beginPath();
    ctx.arc(s.x, s.y, 5, 0, Math.PI * 2);
    ctx.fill();

  }, [trajectory, pos]);

  // =========================================================
  // RENDER (UNCHANGED)
  // =========================================================
return (
  <WebGUIContainer>
    <div
      style={{
        display: "flex",
        width: "100%",
        height: "100%",   // 🔥 IMPORTANTE: NO 100vh
        overflow: "hidden",
      }}
    >

      {/* ================= LEFT IMAGE ================= */}
      <div
        style={{
          flex: 1,
          minHeight: 0,     // 🔥 CLAVE FLEX FIX
          display: "flex",
          background: "#000",
          overflow: "hidden",
          borderRight: "2px solid #333",
        }}
      >
        <div
          style={{
            flex: 1,
            minHeight: 0,
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            overflow: "hidden",
          }}
        >
          {image && (
            <WebGUIImage
              id="gui_canvas"
              src={image}
              style={{
                maxWidth: "100%",
                maxHeight: "100%",
                objectFit: "contain",
              }}
              fit
            />
          )}
        </div>
      </div>

      {/* ================= RIGHT ODOMETRY ================= */}
      <div
        style={{
          flex: 1,
          minHeight: 0,     // 🔥 CLAVE FLEX FIX
          display: "flex",
          flexDirection: "column",
          backgroundColor: "#111",
          color: "white",
          padding: "10px",
          fontFamily: "monospace",
          overflow: "hidden", // 🔥 evita overflow visual
        }}
      >
        <h3 style={{ margin: "0 0 10px 0" }}>Visual Odometry</h3>

        <div style={{ marginBottom: "10px" }}>
          <p><b>X:</b> {pos.x.toFixed(2)}</p>
          <p><b>Y:</b> {pos.y.toFixed(2)}</p>
          <p><b>Drift:</b> {drift.toFixed(3)}</p>
        </div>

        {/* CANVAS AREA */}
        <div
          style={{
            flex: 1,
            minHeight: 0,   // 🔥 MUY IMPORTANTE
            overflow: "hidden",
          }}
        >
          <canvas
            ref={canvasRef}
            width={400}
            height={400}
            style={{
              width: "100%",
              height: "100%",
              display: "block",
              background: "black",
              border: "1px solid #444",
            }}
          />
        </div>
      </div>

    </div>
  </WebGUIContainer>
);
}

export default WebGUI;