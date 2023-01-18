import React, { useContext, useState } from "react";
import { FormControl, InputLabel, MenuItem, Select } from "@mui/material";
import PropTypes from "prop-types";
import RAMCanvasBirdEye from "./RAMCanvasBirdEye";

export default function CircuitSelector(props) {
  const { terminate, doLaunch } = useContext(props.context);
  const [circuit, setCircuit] = useState("default");

  const handleCircuitChange = () => {
    terminate().then(() => {
      const config = JSON.parse(
        document.getElementById("exercise-config").textContent
      );

      // Setting up circuit name into configuration
      config.application.params = { circuit: circuit };
      let launch_file = config.launch["0"].launch_file.interpolate({
        circuit: circuit,
      });
      config.launch["0"].launch_file = launch_file;
      console.log(config, "config");
      doLaunch();
    });
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
            setCircuit(e.target.value);
            handleCircuitChange();
          }}
        >
          <MenuItem value={"default"}>Default Line</MenuItem>
          <MenuItem value={"montmelo"}>Montmelo Line</MenuItem>
          <MenuItem value={"montreal"}>Montreal Line</MenuItem>
          <MenuItem value={"nbg"}>Nürburgring Line</MenuItem>
        </Select>
      </FormControl>
      <RAMCanvasBirdEye circuit={circuit} />
    </>
  );
}

CircuitSelector.propTypes = {
  context: PropTypes.any,
};
