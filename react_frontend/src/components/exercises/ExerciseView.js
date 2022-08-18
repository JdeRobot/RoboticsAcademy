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

function ExerciseView() {
  return (
    <Box id="exercise-view">
      <ExerciseControl />
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
        <AceEditorRobot />
        <Typography align={"center"} m={2} color={"primary"} variant={"h4"}>
          Visualization
        </Typography>
        <CircuitSelector />
        <CanvasBirdEye />
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
          <GazeboViewer />
          <VncConsoleViewer />
        </Box>
      </Box>
      <LoadModalView />
      <CustomAlert />
      <ErrorModalView />
    </Box>
  );
}

export default ExerciseView;
