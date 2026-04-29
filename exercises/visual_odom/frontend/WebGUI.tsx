import React, { useState, useEffect } from "react";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import { useExercise } from "Contexts/ExerciseContext";
import { states } from "jderobot-commsmanager";

function WebGUI() {
  const exerciseContext = useExercise();

  const [image, setImage] = useState<string | undefined>(undefined);

  // =========================
  // POSE + TRAJECTORY
  // =========================
  const [pose, setPose] = useState<any>(null);
  const [trajectory, setTrajectory] = useState<any[]>([]);

  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;
    const update = data.update;

    // =========================
    // IMAGE
    // =========================
    if (update.image) {
      const img = JSON.parse(update.image);
      setImage(`data:image/png;base64,${img.image}`);
    }

    // =========================
    // POSE
    // =========================
    if (update.pose) {
      setPose(update.pose);
    }

    // =========================
    // TRAJECTORY
    // =========================
    if (update.trajectory) {
      setTrajectory(update.trajectory);
    }
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setImage(undefined);
      setPose(null);
      setTrajectory([]);
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  return (
    <WebGUIContainer>

      {/* IMAGE */}
      <WebGUIImage
        id="gui_canvas"
        src={image}
        style={{ width: "100%" }}
        fit
      />

      {/* =========================
          POSE OVERLAY
          ========================= */}
      {pose && (
        <div style={{
          position: "absolute",
          top: 10,
          right: 10,
          background: "rgba(0,0,0,0.6)",
          color: "white",
          padding: 10,
          fontSize: 12,
          borderRadius: 8
        }}>
          <div><b>POSE</b></div>
          <div>x: {pose.x.toFixed(2)}</div>
          <div>y: {pose.y.toFixed(2)}</div>
          <div>z: {pose.z.toFixed(2)}</div>
          <div>yaw: {pose.yaw.toFixed(2)}</div>
        </div>
      )}

      {/* =========================
          TRAJECTORY VISUAL (BÁSICO)
          ========================= */}
      {trajectory.length > 0 && (
        <svg
          style={{
            position: "absolute",
            top: 0,
            left: 0,
            width: "100%",
            height: "100%",
            pointerEvents: "none",
          }}
        >
          {trajectory.map((p, i) => {
            if (i === 0) return null;
            const prev = trajectory[i - 1];

            return (
              <line
                key={i}
                x1={prev.x * 50 + 200}
                y1={prev.y * 50 + 200}
                x2={p.x * 50 + 200}
                y2={p.y * 50 + 200}
                stroke="red"
                strokeWidth={2}
              />
            );
          })}
        </svg>
      )}

    </WebGUIContainer>
  );
}

export default WebGUI;