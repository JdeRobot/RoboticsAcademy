import { Typography } from "@mui/material";
import React, { useState } from "react";
import "../../styles/visualizers/Frequencies.css";
import monitor from "../../images/monitoring2.png";

const Frequencies = () => {
  const [frequencies, setFrequencies] = useState({ brain: 0, gui: 0, rtf: 0 });
  const [rosVersion, setRosVersion] = useState(null);
  const [gpuAvaliable, setGpuAvaliable] = useState(false);
  const [showFrequencies, setShowFrequencies] = React.useState(false);
  const [buttonActive, setButtonActive] = React.useState(false);

  const handleToggleFrequencies = () => {
    setButtonActive(!buttonActive);
    setShowFrequencies(!showFrequencies);
  };

  React.useEffect(() => {
    const callback = (message) => {
      const update = message.data.update;
      if (update.brain) {
        setFrequencies(update);
      }
    };

    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    return () => {
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
        callback
      );
    };
  }, []);

  React.useEffect(() => {
    const callback = (message) => {
      if (message.data.ros_version.trim() === "noetic") {
        setRosVersion("ROS Noetic");
      } else if (message.data.ros_version.trim() === "humble") {
        setRosVersion("ROS2 Humble");
      }
      setGpuAvaliable(message.data.gpu_avaliable);
    };
    window.RoboticsExerciseComponents.commsManager.suscribreOnce(
      [window.RoboticsExerciseComponents.commsManager.events.INTROSPECTION],
      callback
    );
  }, []);

  return (
    <>
      <div className={showFrequencies ? "visible" : "hidden"}>
        <Typography title="BRAIN">{frequencies.brain.toFixed(0)}</Typography>
        <Typography>/</Typography>
        <Typography title="RTF">{frequencies.rtf}</Typography>
        <Typography>/</Typography>
        <Typography>{rosVersion}</Typography>
        <Typography>/</Typography>
        <Typography>GPU</Typography>
        <Typography>{gpuAvaliable ? "ON" : "OFF"}</Typography>
      </div>
      <button
        className={`button ${buttonActive ? "toggledColor" : ""}`}
        onClick={handleToggleFrequencies}
        id="toggleButton"
      >
        <img src={monitor} className="monitor"></img>
      </button>
    </>
  );
};

export default Frequencies;
