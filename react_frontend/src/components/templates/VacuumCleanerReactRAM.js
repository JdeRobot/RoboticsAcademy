import { Box } from "@mui/system";
import React from "react";
import { ExerciseProvider } from "../../contexts/NewManagerExerciseContext";
import { ViewProvider } from "../../contexts/ViewContext";
import MainAppBar from "../common/MainAppBar";
import View from "../common/View";
import NewManagerExerciseContext from "../../contexts/NewManagerExerciseContext";
import RAMFollowLineExerciseView from "../views/RAM/RAMFollowLineExerciseView";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";

const VacuumCleanerReactRAM = () => {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <MainAppBar exerciseName={" Vacuum Cleaner RR"} />
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
};

export default VacuumCleanerReactRAM;
