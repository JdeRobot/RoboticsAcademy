import { useState, useEffect } from "react";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import { useExercise } from "Contexts/ExerciseContext";
import { events } from "jderobot-commsmanager";

function WebGUI() {
  const [image, setImage] = useState(undefined);
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);
  let connection = connectApplication();

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  useEffect(() => {
    if (manager === null) {
      return;
    }

    connection.start(manager);

    const callback = (message) => {
      connection.end();
      const update = message.data.update;
      if (update.image) {
        const image = JSON.parse(update.image);
        setImage(`data:image/png;base64,${image.image}`);
      }

      // Send the ACK of the msg
      manager.send("gui", "ack");
    };

    manager.subscribe([events.UPDATE], callback);

    return () => {
      manager.unsubscribe([events.UPDATE], callback);
    };
  }, [manager]);

  return (
    <WebGUIContainer>
      <WebGUIImage id="gui_canvas" src={image} />
    </WebGUIContainer>
  );
}

export default WebGUI;
