import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/NewManagerExerciseContext";
import NewManagerExerciseContext from "../../contexts/NewManagerExerciseContext";
import MainAppBar from "../common/MainAppBar";
import View from "../common/View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";
import RAMFollowPersonExerciseView from "../views/RAM/RAMFollowPersonExerciseView";
import CircuitSelector from "../buttons/RAM/RAMExerciseConfiguration";

function FollowPersonRAM() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
        <MainAppBar
            exerciseName={"Follow Person"}
            specificConfiguration={<CircuitSelector></CircuitSelector>}
          />
          <View
            url={THEORY_URL.FollowPerson}
            exerciseId={
              <RAMFollowPersonExerciseView context={NewManagerExerciseContext} />
            }
          />
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}

export default FollowPersonRAM;
