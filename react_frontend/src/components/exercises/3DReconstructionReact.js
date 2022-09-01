import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/3DreconstructionExerciseContext";
import ProminentAppBar from "./ProminentAppBar";
import _3DReconstructionExerciseContext from "../../contexts/3DreconstructionExerciseContext";
import View from "./View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";
import _3DReconstructionExerciseView from "./3DReconstructionExerciseView";

function _3DReconstructionReact() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <ProminentAppBar
            exerciseName={" 3D reconstruction "}
            context={_3DReconstructionExerciseContext}
          />
          <View
            url={THEORY_URL._3dReconstruction}
            exerciseId={
              <_3DReconstructionExerciseView
                context={_3DReconstructionExerciseContext}
              />
            }
          />
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}

export default _3DReconstructionReact;
