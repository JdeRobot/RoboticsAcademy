import * as React from "react";
import { Box, Typography } from "@mui/material";
import ExerciseControl from "./ExerciseControl";
import AceEditorRobot from "./AceEditorRobot";
import CircuitSelector from "./CircuitSelector";
import CanvasBirdEye from "./CanvasBirdEye";
import GazeboViewer from "./GazeboViewer";
import VncConsoleViewer from "./VncConsoleViewer";
import LoadModalView from "./LoadModalView";
import CustomAlert from "./CustomAlert";
import ErrorModalView from "./ErrorModalView";
import VisualizationComponents from "./VisualizationComponents";
import FrequencyMenu from "./FrequencyMenu";
import GuiCanvas from "./GuiCanvas";
import PropTypes from "prop-types";
import InfoModalView from "./InfoModalView";

function FollowLineExerciseView(props) {
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
        <CircuitSelector context={props.context} />
        <VisualizationComponents>
          <CanvasBirdEye context={props.context} />
          <GuiCanvas />
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
FollowLineExerciseView.propTypes = {
  context: PropTypes.any,
};

export default FollowLineExerciseView;
