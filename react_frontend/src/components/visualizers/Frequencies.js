import { Typography } from "@mui/material";
import React, { useState } from "react";
import "../../styles/visualizers/Frequencies.css";

export const Frequencies = (props) => {
  const [frequencies, setFrequencies] = useState({ brain: 0, gui: 0, rtf: 0 });
  const [rosVersion, setRosVersion] = useState(null);
  const [gpuVendor, setgpuVendor] = useState(false);
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
        setRosVersion(["ROS", "Noetic"]);
      } else {
        let version = message.data.ros_version.trim()
        if (version) {
          setRosVersion(["ROS2", version.charAt(0).toUpperCase() + version.slice(1)]);
        }
      }
      console.log(message.data.gpu_avaliable)
      setgpuVendor(message.data.gpu_avaliable);
    };
    window.RoboticsExerciseComponents.commsManager.suscribreOnce(
      [window.RoboticsExerciseComponents.commsManager.events.INTROSPECTION],
      callback
    );
  }, []);

  return (
    <div className={props.style}>
      <Typography>AF:</Typography>
      <Typography title="AF">{frequencies.brain.toFixed(0)}</Typography>
      <Typography>Hz</Typography>
      <Typography>/</Typography>
      <Typography>RTF:</Typography>
      <Typography title="RTF">{frequencies.rtf}</Typography>
      {rosVersion &&
      <>
      <Typography>/</Typography>
      <Typography>{rosVersion[0]}</Typography>
      <Typography>{rosVersion[1]}</Typography>
      </>
      }
      <Typography>/</Typography>
      <Typography>GPU</Typography>
      <Typography>{gpuVendor}</Typography>
    </div>
  );
};

export default Frequencies