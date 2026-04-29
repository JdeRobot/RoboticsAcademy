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
  // 🧠 NUEVO: POSE STATE
  // =========================
  const [pose, setPose] = useState<any>(null);

  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;
    const update = data.update;

    // =========================
    // IMAGE (igual que antes)
    // =========================
    if (update.image) {
      const image = JSON.parse(update.image);
      setImage(`data:image/png;base64,${image.image}`);

      // =========================
      // 🧠 NUEVO: POSE
      // =========================
      if (image.pose) {
        setPose(image.pose);
      }
    }
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setImage(undefined);
      setPose(null);
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  return (
    <WebGUIContainer>

      {/* Imagen principal (igual que antes) */}
      <WebGUIImage id="gui_canvas" src={image} style={{ width: "100%" }} fit />

      {/* =========================
          🧠 NUEVO: VISOR POSE DEBUG
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

    </WebGUIContainer>
  );
}

export default WebGUI;