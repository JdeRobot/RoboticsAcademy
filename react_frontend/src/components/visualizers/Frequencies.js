import { Typography } from "@mui/material";
import React, { useState } from "react";
import "../../styles/visualizers/Frequencies.css";

export const Frequencies = (props) => {
  const [frequencies, setFrequencies] = useState({ brain: 0, gui: 0, rtf: 0 });
  const [rosVersion, setRosVersion] = useState(null);
  const [gpuAvaliable, setGpuAvaliable] = useState(false);
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
    <div className={props.style}>
      <Typography title="AF">{frequencies.brain.toFixed(0)} Hz</Typography>
      <Typography>/</Typography>
      <Typography title="RTF">{frequencies.rtf}</Typography>
      <Typography>/</Typography>
      <Typography>{rosVersion}</Typography>
      <Typography>/</Typography>
      <Typography>GPU</Typography>
      <Typography>{gpuAvaliable ? "ON" : "OFF"}</Typography>
    </div>
  );
};

export default Frequencies