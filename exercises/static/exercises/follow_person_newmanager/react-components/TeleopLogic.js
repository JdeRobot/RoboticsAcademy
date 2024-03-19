import React, { useContext, useEffect, useState } from "react";
import PropTypes from "prop-types";

const PersonTeleop = () => {
  const [disabled, setDisabled] = useState(true);

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

  return true;
};
PersonTeleop.propTypes = {
  context: PropTypes.any,
};
export default PersonTeleop;
