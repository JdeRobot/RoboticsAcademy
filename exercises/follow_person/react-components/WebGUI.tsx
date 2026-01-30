import { useState, useEffect } from "react";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import { useExercise } from "Contexts/ExerciseContext";
import { states } from "jderobot-commsmanager";

function WebGUI() {
  const exerciseContext = useExercise();
  const [image, setImage] = useState<string | undefined>(undefined);
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  function listen_key() {
    if (manager === null) {
      return;
    }

    window.addEventListener("keypress", function (event) {
      if (event.code === "KeyS") {
        manager.send("gui", "key_s");
      } else if (event.code === "KeyW") {
        manager.send("gui", "key_w");
      } else if (event.code === "KeyA") {
        manager.send("gui", "key_a");
      } else if (event.code === "KeyD") {
        manager.send("gui", "key_d");
      } else if (event.code === "KeyX") {
        manager.send("gui", "key_x");
      }
    });
  }

  const updateCallback = (updateData: unknown) => {
    let data = updateData as any;
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
  listen_key();

  return (
    <WebGUIContainer>
      <WebGUIImage id="gui_canvas" src={image} style={{ width: "100%" }} />
    </WebGUIContainer>
  );
}

export default WebGUI;
