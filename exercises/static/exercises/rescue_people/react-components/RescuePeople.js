import { useState, useEffect } from "react";
import { drawImage, drawLeftImage } from "./helpers/showImagesRescue";
import noImage from "../../assets/img/noImage.png";
import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";

import "./css/GUICanvas.css";
function RescuePeople() {
  const [rightImage, setRightImage] = useState(noImage);
  const [leftImage, setLeftImage] = useState(noImage);
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
        setRightImage(`data:image/png;base64,${image.image_right}`);
      }
      if (update.image_left) {
        image = JSON.parse(update.image_left);
        setLeftImage(`data:image/png;base64,${image.image_left}`);
      }

      manager.send("gui", "ack");
    };

    manager.subscribe(events.UPDATE, callback);

    return () => {
      manager.unsubscribe(events.UPDATE, callback);
    };
  }, [manager]);

  return (
    <div
      style={{
        display: "flex",
        width: "100%",
        height: "100%",
        position: "relative",
      }}
    >
      <img
        className="image"
        id="gui_canvas_left"
        style={{ left: "0" }}
        src={leftImage}
      />
      <img
        className="image"
        id="gui_canvas_right"
        style={{ left: "50%" }}
        src={rightImage}
      />
    </div>
  );
}

export default RescuePeople;
