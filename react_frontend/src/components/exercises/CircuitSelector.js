import * as React from "react";
import { FormControl, InputLabel, MenuItem, Select } from "@mui/material";
import ExerciseContext from "../../contexts/ExerciseContext";

export default function CircuitSelector() {
  const { handleCircuitChange } = React.useContext(ExerciseContext);
  const circuitSelector = React.useRef(null);
  return (
    <FormControl sx={{ m: 1, p: 2 }} color={"secondary"}>
      <InputLabel id={"circuit-selector"}>Circuit</InputLabel>
      <Select
        defaultValue={"default"}
        labelId="circuit-selector"
        label={"Circuit"}
        color={"success"}
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
