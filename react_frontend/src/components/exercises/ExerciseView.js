import * as React from "react";
import { Box } from "@mui/material";
import ExerciseControl from "./ExerciseControl";
import AceEditorRobot from "./AceEditorRobot";
import CircuitSelector from "./CircuitSelector";
import CanvasBirdEye from "./CanvasBirdEye";
import GazeboViewer from "./GazeboViewer";
import VncConsoleViewer from "./VncConsoleViewer";

function ExerciseView() {
  return (
    <Box id="exercise-view">
      <ExerciseControl />
      <AceEditorRobot />
      <CircuitSelector />
      <CanvasBirdEye />
      <GazeboViewer />
      <VncConsoleViewer />
    </Box>
  );
}

export default ExerciseView;
