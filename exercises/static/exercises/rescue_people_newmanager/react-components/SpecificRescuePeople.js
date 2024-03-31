import * as React from "react";
import PropTypes from "prop-types";
import { drawImage, drawLeftImage } from "./helpers/showImagesRescue";

function SpecificRescuePeople(props) {
  const [image, setImage] = React.useState(null);
  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");
    const callback = (message) => {
      if (message.data.update.image) {
        console.log("image_right");
        const image = JSON.parse(message.data.update.image_right);
        if (image.image) {
          drawImage(message.data.update);
        }
      }
      if (message.data.update.image_left) {
        console.log("image_left");
        const image = JSON.parse(message.data.update.image_left);
        if (image.image_left) {
          drawLeftImage(message.data.update);
        }
      }

      // Send the ACK of the imgs
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
    <div style={{ display: "flex", width: "100%", height: "100%" }}>
      <canvas id="gui_canvas_left"></canvas>
      <canvas id="gui_canvas_right"></canvas>
    </div>
  );
}

SpecificRescuePeople.propTypes = {
  circuit: PropTypes.string,
};

export default SpecificRescuePeople;
