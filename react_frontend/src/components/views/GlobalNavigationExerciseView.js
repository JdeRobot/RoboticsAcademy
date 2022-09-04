import * as React from "react";
import { Box, Typography } from "@mui/material";
import ExerciseControl from "../exercises/ExerciseControl";
import AceEditorRobot from "../exercises/AceEditorRobot";
import CircuitSelector from "../exercises/CircuitSelector";
import GazeboViewer from "../exercises/GazeboViewer";
import VncConsoleViewer from "../exercises/VncConsoleViewer";
import LoadModalView from "../exercises/LoadModalView";
import CustomAlert from "../exercises/CustomAlert";
import ErrorModalView from "../exercises/ErrorModalView";
import VisualizationComponents from "../exercises/VisualizationComponents";
import FrequencyMenu from "../exercises/FrequencyMenu";
import ImgCanvas from "../exercises/ImgCanvas";
import PropTypes from "prop-types";
import InfoModalView from "../exercises/InfoModalView";
import CanvasThree from "../exercises/CanvasThree";
import GuiCanvas from "../exercises/GuiCanvas";

function GlobalNavigationExerciseView(props) {
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
        <VisualizationComponents>
          <GuiCanvas context={props.context} />
          <ImgCanvas context={props.context} />
          <FrequencyMenu context={props.context} />
        </VisualizationComponents>
      </Box>
      {/*  </Box>*/}
      {/*</Box>*/}
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
      <InfoModalView context={props.context} />
    </Box>
  );
}
GlobalNavigationExerciseView.propTypes = {
  context: PropTypes.any,
};

export default GlobalNavigationExerciseView;
