import React, { useState, useEffect } from "react";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, { connectApplication } from "Components/exercise/WebGUIContainer";
import { useExercise } from "Contexts/ExerciseContext";
import { states } from "jderobot-commsmanager";

function WebGUI() {
  const exerciseContext = useExercise();
  const [image, setImage] = useState<string | undefined>(undefined);
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  useEffect(() => {
    if (!manager) return;

    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.code) {
        case "KeyW":
          manager.send("gui", "key_w");
          break;
        case "KeyS":
          manager.send("gui", "key_s");
          break;
        case "KeyA":
          manager.send("gui", "key_a");
          break;
        case "KeyD":
          manager.send("gui", "key_d");
          break;
        case "KeyX":
          manager.send("gui", "key_x");
          break;
      }
    };

    const handleKeyUp = (event: KeyboardEvent) => {
      switch (event.code) {
        case "KeyW":
          manager.send("gui", "key_w_up");
          break;
        case "KeyS":
          manager.send("gui", "key_s_up");
          break;
        case "KeyA":
          manager.send("gui", "key_a_up");
          break;
        case "KeyD":
          manager.send("gui", "key_d_up");
          break;
        case "KeyX":
          manager.send("gui", "key_x_up");
          break;
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
    };
  }, [manager]);

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;
    const update = data.update;

    if (update.image) {
      const image = JSON.parse(update.image);
      setImage(`data:image/png;base64,${image.image}`);
    }
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setImage(undefined);
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  return (
    <WebGUIContainer>
      <WebGUIImage id="gui_canvas" src={image} style={{ width: "100%" }} />
    </WebGUIContainer>
  );
}

export default WebGUI;