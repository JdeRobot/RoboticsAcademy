import * as React from "react";
import Toolbar from "@mui/material/Toolbar";
import { Box } from "@mui/material";
import RoboticsTheme from "../../RoboticsTheme";
import PropTypes from "prop-types";
import { SaveButton } from "../../buttons/SaveButton";
import { LoadFileButton } from "../../buttons/LoadFileButton";
import PlayStopButton from "../../buttons/PlayStopButton";
import GazeboButton from "../../buttons/GazeboButton";
import ConsoleButton from "../../buttons/ConsoleButton";
import RAMPlay from "../../buttons/RAM/RAMPlay";
import RAMPause from "../../buttons/RAM/RAMPause";
import RAMReset from "../../buttons/RAM/RAMReset";

function RAMExerciseControl(props) {
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
        </Box>
        <Box id={"robot-control"}>
          <RAMPlay context={props.context}></RAMPlay>
          <RAMPause></RAMPause>
          <RAMReset></RAMReset>
        </Box>
        <Box id={"Sim-console-control"}>
          <GazeboButton context={props.context} />
          <ConsoleButton context={props.context} />
        </Box>
      </Toolbar>
      <PlayStopButton context={props.context} />
    </RoboticsTheme>
  );
}
RAMExerciseControl.propTypes = {
  context: PropTypes.any.isRequired,
};

export default RAMExerciseControl;
