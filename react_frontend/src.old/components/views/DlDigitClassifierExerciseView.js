import { Box, TextField, Typography } from "@mui/material";
import * as React from "react";
import VisualizationComponents from "../common/VisualizationComponents";
import GuiCanvas from "../visualizers/GuiCanvas";
import FrequencyMenu from "../common/FrequencyMenu";
import GazeboViewer from "../exercises/GazeboViewer";
import VncConsoleViewer from "../exercises/VncConsoleViewer";
import LoadModal from "../modals/LoadModal";
import CustomAlert from "../common/CustomAlert";
import ErrorModal from "../modals/ErrorModal";
import PropTypes from "prop-types";
import FileSelector from "../exercises/FileSelector";
import RoboticsTheme from "../RoboticsTheme";
import Toolbar from "@mui/material/Toolbar";
import ResetButton from "../buttons/ResetButton";
import ConsoleButton from "../buttons/ConsoleButton";
import PlayStopButton from "../buttons/PlayStopButton";

export default function DlDigitClassifierExerciseView(props) {
  return (
    <Box id="exercise-view">
      <ExerciseControl context={props.context} />
      <Box
        sx={{
          display: "flex",
          border: "2px solid",
          alignItems: "center",
          flexDirection: "column",
          justifyContent: "space-around",
          p: 1,
          m: 1,
          background: "linear-gradient(#EOECDE, #FFFFFF)",
        }}
      >
        <FileSelector context={props.context} />
        <Typography align={"center"} m={2} color={"primary"} variant={"h4"}>
          Visualization
        </Typography>
        <VisualizationComponents>
          <GuiCanvas context={props.context} />
          <FrequencyMenu context={props.context} />
        </VisualizationComponents>
      </Box>

      <Box
        sx={{
          display: "flex",
          flexDirection: "column",
          border: "2px solid",
          p: 1,
          m: 1,
        }}
      >
        <Typography align={"center"} m={2} color={"primary"} variant={"h4"}>
          {" "}
          Console !{" "}
        </Typography>
        <Box
          sx={{
            display: "flex",
            alignItems: "center",
            justifyContent: "space-around",
            p: 2,
          }}
        >
          <GazeboViewer context={props.context} />
          <VncConsoleViewer context={props.context} />
        </Box>
      </Box>
      <LoadModal context={props.context} />
      <CustomAlert context={props.context} />
      <ErrorModal context={props.context} />
    </Box>
  );
}

DlDigitClassifierExerciseView.propTypes = {
  context: PropTypes.any,
};

function ExerciseControl(props) {
  const { guiFreq, brainFreq, keyHandleFrequency } = React.useContext(
    props.context
  );
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
        <Box id={"freq-control"}>
          <TextField
            id={"code_freq"}
            label="Brain Freq (Hz)"
            type="number"
            value={brainFreq}
            size={"small"}
            title={
              "The brain frequency is the value that tells the robot with what frequency should the code be updated"
            }
            sx={{ width: 160, m: 1 }}
            onKeyDown={keyHandleFrequency}
            color={"secondary"}
          />
          <TextField
            id={"gui_freq"}
            label="GUI Freq (Hz)"
            value={guiFreq}
            type="number"
            sx={{ width: 160, m: 1 }}
            title={
              "This value corresponds to the frequency the UI and visuals will be updated."
            }
            onKeyDown={keyHandleFrequency}
            size={"small"}
            color={"secondary"}
          />
        </Box>
        <Box id={"Sim-console-control"}>
          <ConsoleButton context={props.context} />
        </Box>
      </Toolbar>
      <PlayStopButton context={props.context} />
    </RoboticsTheme>
  );
}

ExerciseControl.propTypes = {
  context: PropTypes.any,
};
