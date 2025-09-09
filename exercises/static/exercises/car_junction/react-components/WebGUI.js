import { useState, useEffect} from "react";
import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIContainer from "Components/exercise/WebGUIContainer";
import WebGUIImage from "Components/exercise/WebGUIImage";

function WebGUI() {
  const [frontImage, setFrontImage] = useState(undefined);
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
      let image;
      if (update.image_front) {
        image = JSON.parse(update.image_front);
        setFrontImage(`data:image/png;base64,${image.image_front}`);
      }
      // Send the ACK of the msg
      manager.send("gui", "ack");
    };

    manager.subscribe(events.UPDATE, callback);

    return () => {
      manager.unsubscribe(events.UPDATE, callback);
    };
  }, [manager]);

   return (
    <WebGUIContainer>
      <WebGUIImage id="front_img" style={{ left: "25%" }} src={frontImage} />
    </WebGUIContainer>
  );
}

export default WebGUI;
