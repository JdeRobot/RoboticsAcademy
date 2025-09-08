import { useState, useEffect } from "react";
import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer from "Components/exercise/WebGUIContainer";

function WebGUI() {
  const [rightImage, setRightImage] = useState(undefined);
  const [leftImage, setLeftImage] = useState(undefined);
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
      if (update.image_right) {
        image = JSON.parse(update.image_right);
        setLeftImage(`data:image/png;base64,${image.image_right}`);
      }
      if (update.image_left) {
        image = JSON.parse(update.image_left);
        setRightImage(`data:image/png;base64,${image.image_left}`);
      }

      manager.send("gui", "ack");
    };

    manager.subscribe(events.UPDATE, callback);

    return () => {
      manager.unsubscribe(events.UPDATE, callback);
    };
  }, [manager]);

  return (
    <WebGUIContainer>
      <WebGUIImage id="left_img" style={{ left: "0" }} src={leftImage} />
      <WebGUIImage id="right_img" style={{ left: "50%" }} src={rightImage} />
    </WebGUIContainer>
  );
}

export default WebGUI;
