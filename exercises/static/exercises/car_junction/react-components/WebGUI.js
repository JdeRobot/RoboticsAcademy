import { useState, useEffect } from "react";
import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer from "Components/exercise/WebGUIContainer";
import "./css/GUICanvas.css";

function WebGUI() {
  const [image, setImage] = useState(undefined);
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  useEffect(() => {
    if (manager === null) {
      return;
    }

    const callback = (message) => {
      const update = message.data.update;
      if (update.image_front) {
        console.log("New img received");
        const image = JSON.parse(update.image_front);
        setImage(`data:image/png;base64,${image.image}`);

        // Send the ACK of the img
        manager.send("gui", "ack");
      }
    };

    listen_key();
    manager.subscribe(events.UPDATE, callback);

    return () => {
      manager.unsubscribe(events.UPDATE, callback);
    };
  }, [manager]);

  return (
    <WebGUIContainer>
      <WebGUIImage id="gui_canvas" src={image} />
    </WebGUIContainer>
  );
}

export default WebGUI;
