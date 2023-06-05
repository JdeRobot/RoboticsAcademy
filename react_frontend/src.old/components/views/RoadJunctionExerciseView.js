import * as React from "react";
import { Box, Typography } from "@mui/material";
import ExerciseControl from "../common/ExerciseControl";
import AceEditorRobot from "../exercises/AceEditorRobot";
import GazeboViewer from "../exercises/GazeboViewer";
import VncConsoleViewer from "../exercises/VncConsoleViewer";
import LoadModal from "../modals/LoadModal";
import CustomAlert from "../common/CustomAlert";
import ErrorModal from "../modals/ErrorModal";
import VisualizationComponents from "../common/VisualizationComponents";
import FrequencyMenu from "../common/FrequencyMenu";
import PropTypes from "prop-types";
import InfoModal from "../modals/InfoModal";
import GuiCanvas from "../visualizers/GuiCanvas";

function RoadJunctionExerciseView(props) {
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
          <GuiCanvas context={props.context} drone={false} />
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
      <LoadModal context={props.context} />
      <CustomAlert context={props.context} />
      <ErrorModal context={props.context} />
      <InfoModal context={props.context} />
    </Box>
  );
}

RoadJunctionExerciseView.propTypes = {
  context: PropTypes.any,
};

export default RoadJunctionExerciseView;
