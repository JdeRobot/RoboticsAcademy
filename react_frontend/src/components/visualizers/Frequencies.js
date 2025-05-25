import { Typography } from "@mui/material";
import React, { useState } from "react";
import "../../styles/visualizers/Frequencies.css";

export const Frequencies = (props) => {
  const [frequencies, setFrequencies] = useState({
    brain: 0,
    gui: 0,
    rtf: -1,
    fps: -1,
    lat: -1,
  });
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
      let version = message.data.ros_version.trim();
      if (version) {
        setRosVersion([
          "ROS2",
          version.charAt(0).toUpperCase() + version.slice(1),
        ]);
      }
      console.log(message.data.gpu_avaliable);
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
      {frequencies.rtf >= 0 && (
        <>
          <Typography>/</Typography>
          <Typography>RTF:</Typography>
          <Typography title="RTF">{frequencies.rtf}</Typography>
        </>
      )}
      {frequencies.fps >= 0 && (
        <>
          <Typography>/</Typography>
          <Typography>FPS:</Typography>
          <Typography title="FPS">
            {frequencies.fps < 10
              ? frequencies.fps === 0
                ? `0`
                : `0${frequencies.fps}`
              : frequencies.fps}
          </Typography>
        </>
      )}
      {frequencies.lat >= 0 && (
        <>
          <Typography>/</Typography>
          <Typography>LATENCY:</Typography>
          <Typography title="LATENCY">
            {frequencies.lat >= 1000
              ? `${(frequencies.lat / 1000).toFixed(0)} s`
              : `${frequencies.lat} ms`}
          </Typography>
        </>
      )}
      {rosVersion && (
        <>
          <Typography>/</Typography>
          <Typography>{rosVersion[0]}</Typography>
          <Typography>{rosVersion[1]}</Typography>
        </>
      )}
      <Typography>/</Typography>
      <Typography>GPU</Typography>
      <Typography>{gpuVendor}</Typography>
    </div>
  );
};

export default Frequencies;
