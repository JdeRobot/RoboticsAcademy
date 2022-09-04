import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/VacuumCleanerExerciseContext";
import ProminentAppBar from "../exercises/ProminentAppBar";
import VacuumCleanerExerciseContext from "../../contexts/VacuumCleanerExerciseContext";
import View from "../exercises/View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";

import VacuumCleanerExerciseView from "../views/VacuumCleanerExerciseView";

function _3DReconstructionReact() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <ProminentAppBar
            exerciseName={" Vacuum Cleaner "}
            context={VacuumCleanerExerciseContext}
          />
          <View
            url={THEORY_URL.VacuumCleaner}
            exerciseId={
              <VacuumCleanerExerciseView
                context={VacuumCleanerExerciseContext}
              />
            }
          />
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}

export default _3DReconstructionReact;
