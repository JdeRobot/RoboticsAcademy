import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/NewManagerExerciseContext";
import NewManagerExerciseContext from "../../contexts/NewManagerExerciseContext";
import MainAppBar from "../common/MainAppBar";
import View from "../common/View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";
import RAMFollowLineExerciseView from "../views/RAM/RAMFollowLineExerciseView";
import CircuitSelector from "../buttons/RAM/RAMExerciseConfiguration";

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
