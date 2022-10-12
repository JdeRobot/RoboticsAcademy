import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/AutoParkingExerciseContext";
import MainAppBar from "../common/MainAppBar";
import AutoParkingExerciseContext from "../../contexts/AutoParkingExerciseContext";
import View from "../common/View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";
import AutoParkingExerciseView from "../views/AutoParkingExerciseView";

function AutoParkingReact() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <MainAppBar
            exerciseName={" Auto Parking "}
            context={AutoParkingExerciseContext}
          />
          <View
            url={THEORY_URL.AutoParking}
            exerciseId={
              <AutoParkingExerciseView
                context={AutoParkingExerciseContext}
              />
            }
          />
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}

export default AutoParkingReact;
