import * as React from "react";
import PropTypes from "prop-types";
import noImage from "../../assets/img/noImage.png";
import "./css/GUICanvas.css";

export default function ImgCanvas() {
  const [image, setImage] = React.useState(
    noImage
  );

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");

    const callback = (message) => {
      const update = message.data.update;
      if (update.image) {
        console.log("New img received");
        const image = JSON.parse(update.image);
        setImage(`data:image/png;base64,${image.image}`);

        // Send the ACK of the img
        window.RoboticsExerciseComponents.commsManager.send("gui", "ack");
      }
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
    <div style={{display: "flex", width: "100%", height: "100%", position:"relative", justifyContent: "center"}}>
      <img className="image" id="gui_canvas" src={image}/>
    </div>
  );
}

ImgCanvas.propTypes = {
  context: PropTypes.any,
};
