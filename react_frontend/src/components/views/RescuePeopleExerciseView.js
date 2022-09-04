import { Box, Typography } from "@mui/material";
import * as React from "react";
import ExerciseControl from "../exercises/ExerciseControl";
import AceEditorRobot from "../exercises/AceEditorRobot";
import VisualizationComponents from "../exercises/VisualizationComponents";
import GuiCanvas from "../exercises/GuiCanvas";
import FrequencyMenu from "../exercises/FrequencyMenu";
import GazeboViewer from "../exercises/GazeboViewer";
import VncConsoleViewer from "../exercises/VncConsoleViewer";
import LoadModalView from "../exercises/LoadModalView";
import CustomAlert from "../exercises/CustomAlert";
import ErrorModalView from "../exercises/ErrorModalView";
import PropTypes from "prop-types";
import CanvasThree from "../exercises/CanvasThree";

export default function RescuePeopleExerciseView(props) {
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
        <Typography align={"center"} color={"primary"} variant={"h4"}>
          {" "}
          Start Coding !{" "}
        </Typography>
        <AceEditorRobot context={props.context} />
        <Typography align={"center"} m={2} color={"primary"} variant={"h4"}>
          Visualization
        </Typography>
        <CanvasThree context={props.context} />
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
          Simulation and Console !{" "}
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
      <LoadModalView context={props.context} />
      <CustomAlert context={props.context} />
      <ErrorModalView context={props.context} />
    </Box>
  );
}

RescuePeopleExerciseView.propTypes = {
  context: PropTypes.any,
};
