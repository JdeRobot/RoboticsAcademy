import { useState, useEffect} from "react";
import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import { drawImage } from "./helpers/showImagesCarJunction";
import noImage from "../../assets/img/noImage.png";

import "./css/GUICanvas.css"
function WebGUI() {
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
      console.log(message);

      if (message.data.update.image_front) {
        console.log("image_front");
        drawImage(message.data.update);
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
    <div style={{display: "flex", width: "100%", height: "100%",justifyContent: "center", alignItems: "center"}}>
      <img className="image" id="gui_canvas_front" style={{ maxWidth: "100%", maxHeight: "100%" }}
        src={noImage}/>
    </div>
  );
}

export default WebGUI;
