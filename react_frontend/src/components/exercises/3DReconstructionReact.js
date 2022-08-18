import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/ExerciseContext";
import ProminentAppBar from "./ProminentAppBar";
import View from "./View";

function _3DReconstructionReact() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <ProminentAppBar />
          <View
            url={
              "https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/follow_line/"
            }
          />
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}

export default _3DReconstructionReact;
