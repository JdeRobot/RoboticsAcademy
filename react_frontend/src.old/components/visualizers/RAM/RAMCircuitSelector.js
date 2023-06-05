import React, { useContext, useState } from "react";
import { FormControl, InputLabel, MenuItem, Select } from "@mui/material";
import PropTypes from "prop-types";

export default function CircuitSelector(props) {
  const { terminate, doLaunch } = useContext(props.context);
  const [setCircuit] = useState("default");

  const handleCircuitChange = (circuitPath) => {
    setCircuit(circuitPath);
    terminate().then(() => {
      const config = changeConfig(circuitPath);
      doLaunch(config);
    });
  };

  const changeConfig = (circuitPath) => {
    const config = JSON.parse(
      document.getElementById("exercise-config").textContent
    );
    config.application.params = { circuit: circuitPath };
    config.launch[
      "0"
    ].launch_file = `$EXERCISE_FOLDER/launch/simple_line_follower_ros_headless_${circuitPath}.launch`;
    return config;
  };

  return (
    <>
      <FormControl variant={"filled"} m={3} fullWidth>
        <InputLabel id={"circuit-selector-label"}>Circuit</InputLabel>
        <Select
          defaultValue={"default"}
          labelId="circuit-selector-label"
          id={"circuit-selector"}
          color={"success"}
          label={"Circuit"}
          sx={{
            textAlign: "left",
          }}
          onChange={(e) => {
            handleCircuitChange(e.target.value);
          }}
        >
          <MenuItem value={"default"}>Default Line</MenuItem>
          <MenuItem value={"montmelo"}>Montmelo Line</MenuItem>
          <MenuItem value={"montreal"}>Montreal Line</MenuItem>
          <MenuItem value={"nbg"}>Nürburgring Line</MenuItem>
        </Select>
      </FormControl>
    </>
  );
}

CircuitSelector.propTypes = {
  context: PropTypes.any,
};
