import * as React from "react";
import { FormControl, InputLabel, MenuItem, Select } from "@mui/material";
import ExerciseContext from "../../contexts/ExerciseContext";

export default function CircuitSelector() {
  const { handleCircuitChange } = React.useContext(ExerciseContext);
  const circuitSelector = React.useRef(null);
  return (
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
        ref={circuitSelector}
        onChange={(e) => handleCircuitChange(e, circuitSelector)}
      >
        <MenuItem value={"default"}>Default Line</MenuItem>
        <MenuItem value={"montmelo"}>Montmelo Line</MenuItem>
        <MenuItem value={"montreal"}>Montreal Line</MenuItem>
        <MenuItem value={"nbg"}>Nürburgring Line</MenuItem>
      </Select>
    </FormControl>
  );
}
