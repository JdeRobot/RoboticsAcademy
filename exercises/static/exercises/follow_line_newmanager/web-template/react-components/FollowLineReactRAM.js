import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "Contexts/ViewContext";
import { ExerciseProvider } from "Contexts/NewManagerExerciseContext";
import NewManagerExerciseContext from "Contexts/NewManagerExerciseContext";
import MainAppBar from "Components/common/MainAppBar";
import View from "Components/common/View";
import { THEORY_URL } from "Helpers/TheoryUrlGetter";
import RAMFollowLineExerciseView from "Components/views/RAM/RAMFollowLineExerciseView";

function FollowLineReactRAM() {
  return (
    <Box>
      <ViewProvider>
        <>
          <MainAppBar exerciseName={" Follow Line RR"} />
          <View
            url={THEORY_URL.FollowLine}
            exerciseId={
              <RAMFollowLineExerciseView context={NewManagerExerciseContext} />
            }
          />
        </>
      </ViewProvider>
    </Box>
  );
}

export default FollowLineReactRAM;
