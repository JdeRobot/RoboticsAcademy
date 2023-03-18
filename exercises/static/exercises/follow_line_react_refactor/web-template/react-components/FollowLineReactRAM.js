import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "Contexts/ViewContext";
import { ExerciseProvider } from "Contexts/NewManagerExerciseContext";
import NewManagerExerciseContext from "Contexts/NewManagerExerciseContext";
import MainAppBar from "Components/common/MainAppBar";
import View from "Components/common/View";
import { THEORY_URL } from "Helpers/TheoryUrlGetter";
import RAMFollowLineExerciseView from "Components/views/RAM/RAMFollowLineExerciseView";
import CircuitSelector from "Components/buttons/RAM/RAMExerciseConfiguration";

function FollowLineReactRAM() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <MainAppBar
            exerciseName={" Follow Line RR"}
            specificConfiguration={<CircuitSelector></CircuitSelector>}
          />
          <View
            url={THEORY_URL.FollowLine}
            exerciseId={
              <RAMFollowLineExerciseView context={NewManagerExerciseContext} />
            }
          />
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}

export default FollowLineReactRAM;
