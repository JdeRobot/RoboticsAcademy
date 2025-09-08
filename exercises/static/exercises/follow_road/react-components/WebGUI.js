import { useState, useEffect} from "react";
import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import { drawImage, drawLeftImage } from "./helpers/showImagesFollowRoad";
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

      if (message.data.update.image_right) {
        console.log("image_right");
        drawImage(message.data.update);
      }
      if (message.data.update.image_left) {
        console.log("image_left");
        drawLeftImage(message.data.update);
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
    <div style={{display: "flex", width: "100%", height: "100%", position:"relative"}}>
      <img className="image" id="gui_canvas_left" style={{left: "0"}}
        src={noImage}/>
      <img className="image" id="gui_canvas_right" style={{left: "50%"}}
        src={noImage}/>
    </div>
  );
}

export default WebGUI;
