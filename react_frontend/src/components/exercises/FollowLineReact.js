import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/ExerciseContext";
import ProminentAppBar from "./ProminentAppBar";
import View from "./View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";

function FollowLineReact() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <ProminentAppBar />
          <View url={THEORY_URL.FollowLine} />
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}

export default FollowLineReact;
