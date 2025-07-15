import React, { useEffect, useState } from "react";
import noImage from "../../assets/img/noImage.png";
import "./css/GUICanvas.css";
import PropTypes from "prop-types";

function SpecificFollowPerson(props) {
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

  

  useEffect(() => {
    listen_key();
  }, []);

  function listen_key() {
    window.addEventListener("keypress", function (event) {
      if (event.code === "KeyS") {
        window.RoboticsExerciseComponents.commsManager.send("gui", "key_s");
      } else if (event.code === "KeyW") {
        window.RoboticsExerciseComponents.commsManager.send("gui", "key_w");
      } else if (event.code === "KeyA") {
        window.RoboticsExerciseComponents.commsManager.send("gui", "key_a");
      } else if (event.code === "KeyD") {
        window.RoboticsExerciseComponents.commsManager.send("gui", "key_d");
      } else if (event.code === "KeyX") {
        window.RoboticsExerciseComponents.commsManager.send("gui", "key_x");
      }
    });
  }

  return (
    <div style={{display: "flex", width: "100%", height: "100%", position:"relative", justifyContent: "center"}}>
      <img className="image" id="gui_canvas" src={image}/>
    </div>
  );

}

SpecificFollowPerson.propTypes = {
  context: PropTypes.any,
};
export default SpecificFollowPerson;