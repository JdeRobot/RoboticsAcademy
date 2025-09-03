import { Typography } from "@mui/material";
import React, { useState } from "react";
import "../../styles/visualizers/Frequencies.css";
import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";

type FrequenciesProps = {
  style?: string;
};

type FrequenciesData = {
  brain: number;
  gui: number;
  rtf: number;
  fps: number;
  lat: number;
};

const Frequencies: React.FC<FrequenciesProps> = ({ style }) => {
  const exerciseContext = useExercise();
  const [frequencies, setFrequencies] = useState<FrequenciesData>({
    brain: 0,
    gui: 0,
    rtf: -1,
    fps: -1,
    lat: -1,
  });
  const [rosVersion, setRosVersion] = useState<[string, string] | null>(null);
  const [gpuVendor, setgpuVendor] = useState<boolean>(false);
  React.useEffect(() => {
    if (exerciseContext.manager === null) {
      return;
    }

    const callback = (message: MessageEvent<any>) => {
      const update = message.data.update;
      if (update.brain) {
        setFrequencies(update);
      }
    };

    exerciseContext.manager.subscribe([events.UPDATE], callback);

    return () => {
      if (exerciseContext.manager === null) {
        return;
      }
      exerciseContext.manager.unsubscribe([events.UPDATE], callback);
    };
  }, []);

  React.useEffect(() => {
    if (exerciseContext.manager === null) {
      return;
    }
    const callback = (message: MessageEvent<any>) => {
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
    exerciseContext.manager.subscribeOnce([events.INTROSPECTION], callback);
  }, []);

  return (
    <div className={style}>
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
