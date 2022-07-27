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

function ExerciseView() {
  return (
    <Box id="exercise-view">
      <ExerciseControl />
      <Box
        sx={{
          display: "flex",
          border: "2px solid #d3d3d3",
          alignItems: "center",
          flexDirection: "column",
          justifyContent: "space-around",
          p: 1,
          m: 1,
        }}
      >
        <Typography align={"center"}> Code and Visualize !!! </Typography>
        <Box
          sx={{
            display: "flex",
            alignItems: "center",
          }}
        >
          <AceEditorRobot />
          <Box
            sx={{
              display: "flex",
              flexDirection: "column",
              border: "2px solid #d3d3d3",
              m: 3,
              p: 2,
            }}
          >
            <Typography align={"center"}>Visualization</Typography>
            <CircuitSelector />
            <CanvasBirdEye />
          </Box>
        </Box>
      </Box>
      <Box
        sx={{
          display: "flex",
          flexDirection: "column",
          border: "2px solid #d3d3d3",
          p: 1,
          m: 1,
        }}
      >
        <Typography align={"center"}> Simulation and Console !!! </Typography>
        <Box
          sx={{
            display: "flex",
            border: "2px solid #d3d3d3",
            alignItems: "center",
            justifyContent: "space-around",
            p: 2,
            m: 3,
          }}
        >
          <GazeboViewer />
          <VncConsoleViewer />
        </Box>
      </Box>
      <LoadModalView />
      <CustomAlert />
    </Box>
  );
}

export default ExerciseView;
