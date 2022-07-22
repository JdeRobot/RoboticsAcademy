import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/ExerciseContext";
import ProminentAppBar from "./ProminentAppBar";
import View from "./View";
import { WebSocketProvider } from "../../contexts/WebSocketContext";

function FollowLineReact() {
  return (
    <Box>
      <ViewProvider>
        <WebSocketProvider>
          <ExerciseProvider>
            <ProminentAppBar />
            <View />
          </ExerciseProvider>
        </WebSocketProvider>
      </ViewProvider>
    </Box>
  );
}

export default FollowLineReact;
