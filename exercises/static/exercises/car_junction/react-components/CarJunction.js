import * as React from "react";
import PropTypes from "prop-types";
import { drawImage } from "./helpers/showImagesCarJunction";
import noImage from "../../assets/img/noImage.png";

import "./css/GUICanvas.css"
function CarJunction(props) {
  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");
    const callback = (message) => {
      console.log(message);

      if (message.data.update.image_front) {
        console.log("image_front");
        drawImage(message.data.update);
      }
      // Send the ACK of the msg
      window.RoboticsExerciseComponents.commsManager.send("gui", "ack");
    };

    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    return () => {
      console.log("TestShowScreen unsubscribing from ['state-changed'] events");
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
        callback
      );
    };
  }, []);

  return (
    <div style={{display: "flex", width: "100%", height: "100%",justifyContent: "center", alignItems: "center"}}>
      <img className="image" id="gui_canvas_front" style={{ maxWidth: "100%", maxHeight: "100%" }}
        src={noImage}/>
    </div>
  );
}

CarJunction.propTypes = {
  circuit: PropTypes.string,
};

export default CarJunction;
