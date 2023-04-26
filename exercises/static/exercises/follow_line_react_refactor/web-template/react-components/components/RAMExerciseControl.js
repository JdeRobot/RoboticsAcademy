import * as React from "react";
import Toolbar from "@mui/material/Toolbar";
import {Box, TextField} from "@mui/material";
import RoboticsTheme from "Components/RoboticsTheme";
import PropTypes from "prop-types";
import {SaveButton} from "./buttons/SaveButton";
import {LoadFileButton} from "./buttons/LoadFileButton";
import GazeboButton from "Components/buttons/GazeboButton";
import ConsoleButton from "Components/buttons/ConsoleButton";
import RAMStop from "Components/buttons/RAM/RAMStop";
import CameraButton from "Components/buttons/RAMVisualizatorButton";
import {Frequencies} from "Components/visualizers/RAM/RAMFrequency";
import PlayPause from "./buttons/RAMPlayPause";
import {useState} from "react";

import {StyledEngineProvider} from '@mui/material/styles';
import "../css/RamExerciseControl.css";

function RAMExerciseControl(props) {
  const [editorRendered, setEditorRendered] = React.useState(false);

  React.useEffect(() => {
    if(document.getElementById("code-container")) {
      setEditorRendered(true);
    }
  });

  return (
    <RoboticsTheme>
      <Toolbar className={"exercise-toolbar"}>
        {editorRendered ?
          <Box id={"editor-control"}>
            <LoadFileButton/>
            <SaveButton/>
          </Box> : null
        }
        <Box id={"robot-control"}
             sx={{
               display: "flex",
               flexWrap: "wrap",
               justifyContent: "space-between",
               alignItems: "center",
               m: 1,
             }}
        >
        <PlayPause></PlayPause>
        {/*
        <RAMStop></RAMStop>
        <Frequencies></Frequencies>
        */}
        </Box>
        <Box id={"sim-console-control"}>
          {/*
        <GazeboButton />
        <CameraButton></CameraButton>
        <ConsoleButton />
        */}
        </Box>
      </Toolbar>
    </RoboticsTheme>
  );
};

RAMExerciseControl.propTypes = {
  specificConfiguration: PropTypes.any,
};

export default RAMExerciseControl;
