import * as React from "react";
import PropTypes from "prop-types";
import { drawImage, drawLeftImage } from "./helpers/showImagesRescue";

import "./css/GUICanvas.css"
function SpecificRescuePeople(props) {
  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");
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
    <div style={{display: "flex", width: "100%", height: "100%", position:"relative"}}>
      <img className="image" id="gui_canvas_left" style={{left: "0"}}
        src="https://via.placeholder.com/800x600.png?text=No%20image%20received%20from%20exercise"/>
      <img className="image" id="gui_canvas_right" style={{left: "50%"}}
        src="https://via.placeholder.com/800x600.png?text=No%20image%20received%20from%20exercise"/>
    </div>
  );
}

SpecificRescuePeople.propTypes = {
  circuit: PropTypes.string,
};

export default SpecificRescuePeople;
