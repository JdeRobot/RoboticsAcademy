import { useState, useEffect } from "react";
import { events, states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";

function WebGUI() {
  const [rightImage, setRightImage] = useState(undefined);
  const [leftImage, setLeftImage] = useState(undefined);
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);
  let connection = connectApplication(manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  useEffect(() => {
    if (manager === null) {
      return;
    }

    

    const updateCallback = (message) => {
      connection.end();
      const update = message.data.update;
      let image;
      if (update.image_right) {
        image = JSON.parse(update.image_right);
        if (image.image_right != "" && image.shape_right instanceof Array) {
          setRightImage(`data:image/png;base64,${image.image_right}`);
        }
      }
      if (update.image_left) {
        image = JSON.parse(update.image_left);
        if (image.image_left != "" && image.shape_left instanceof Array) {
          setLeftImage(`data:image/png;base64,${image.image_left}`);
        }
      }

      manager.send("gui", "ack");
    };

    const stateCallback = (message) => {
      if (message.data.state === states.TOOLS_READY) {
        setLeftImage();
        setRightImage();
      }
    };


    manager.subscribe(events.UPDATE, updateCallback);
    manager.subscribe(events.STATE_CHANGED, stateCallback);

    return () => {
      manager.unsubscribe(events.UPDATE, updateCallback);
      manager.unsubscribe(events.STATE_CHANGED, stateCallback);
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
