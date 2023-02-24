import React, { useEffect, useState } from "react";

import MenuItem from "@mui/material/MenuItem";
import SettingsIcon from "@mui/icons-material/Settings";

import { FormControl, InputLabel, Select } from "@mui/material";

export default function CircuitSelector() {
  const changeConfig = (circuitPath) => {
    const config = JSON.parse(
      document.getElementById("exercise-config").textContent
    );
    config.application.params = { circuit: circuitPath };
    config.launch[
      "0"
    ].launch_file = `$EXERCISE_FOLDER/web-template/launch/simple_line_follower_ros_headless_${circuitPath}.launch`;
    return config;
  };

  const handleCircuitChange = (e) => {
    const config = changeConfig(e);
    window.RoboticsExerciseComponents.commsManager.terminate().then(() => {
      window.RoboticsExerciseComponents.commsManager.launch(config);
    });
  };

  const [disabled, setDisabled] = useState(true);

  useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "ready") {
        setDisabled(false);
      } else {
        setDisabled(true);
      }
    };
    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
      callback
    );

    return () => {
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
        callback
      );
    };
  }, []);

  return (
    <>
      <FormControl>
        <InputLabel id={"circuit-selector-label"}>
          <SettingsIcon></SettingsIcon>
        </InputLabel>
        <Select
          disabled={disabled}
          defaultValue={"default"}
          labelId="circuit-selector-label"
          id={"circuit-selector"}
          label={"Circuit"}
          onChange={(e) => {
            handleCircuitChange(e.target.value);
          }}
        >
          <MenuItem value={"default"}>Default</MenuItem>
          <MenuItem value={"montmelo"}>Montmelo</MenuItem>
          <MenuItem value={"montreal"}>Montreal</MenuItem>
          <MenuItem value={"nbg"}>Nürburgring</MenuItem>
        </Select>
      </FormControl>
    </>
  );
}
