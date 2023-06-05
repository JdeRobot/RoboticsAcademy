import * as React from "react";
import Toolbar from "@mui/material/Toolbar";
import { Box, TextField } from "@mui/material";
import RoboticsTheme from "../../RoboticsTheme";
import PropTypes from "prop-types";
import { SaveButton } from "../../buttons/SaveButton";
import { LoadFileButton } from "../../buttons/LoadFileButton";
import GazeboButton from "../../buttons/GazeboButton";
import ConsoleButton from "../../buttons/ConsoleButton";
import RAMStop from "../../buttons/RAM/RAMStop";
import CameraButton from "../../buttons/RAMVisualizatorButton";
import { Frequencies } from "../../visualizers/RAM/RAMFrequency";
import { PlayPause } from "../../buttons/RAM/RAMPlayPause";

function RAMExerciseControl(props) {
  const { filename, setFileName } = React.useContext(props.context);

  return (
    <RoboticsTheme>
      <Toolbar
        sx={{
          display: "flex",
          flexWrap: "wrap",
          justifyContent: "space-between",
          alignItems: "center",
          m: 1,
          border: "2px solid #d3d3d3",
        }}
      >
        <Box id={"editor-control"}>
          <LoadFileButton context={props.context} />
          <SaveButton context={props.context} />
          <TextField
            sx={{ m: "6px" }}
            size={"small"}
            id="filename"
            label="Filename"
            color={"secondary"}
            value={filename}
            onChange={(e) => {
              setFileName(e.target.value);
            }}
          />
        </Box>
        <Box
          id={"robot-control"}
          sx={{
            display: "flex",
            flexWrap: "wrap",
            justifyContent: "space-between",
            alignItems: "center",
            m: 1,
          }}
        >
          <PlayPause context={props.context}></PlayPause>
          <RAMStop></RAMStop>
          <Frequencies></Frequencies>
        </Box>
        <Box id={"Sim-console-control"}>
          <GazeboButton context={props.context} />
          <CameraButton context={props.context}></CameraButton>
          <ConsoleButton context={props.context} />
        </Box>
      </Toolbar>
    </RoboticsTheme>
  );
}
RAMExerciseControl.propTypes = {
  context: PropTypes.any.isRequired,
  specificConfiguration: PropTypes.any,
};

export default RAMExerciseControl;
