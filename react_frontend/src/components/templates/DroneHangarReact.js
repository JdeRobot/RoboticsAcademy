import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/DroneHangarExerciseContext";
import MainAppBar from "../common/MainAppBar";
import DroneHangarExerciseContext from "../../contexts/DroneHangarExerciseContext";
import View from "../common/View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";
import DroneHangarExerciseView from "../views/DroneHangarExerciseView";

export default function DroneHangarReact() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <MainAppBar
            exerciseName={" Drone Hangar "}
            context={DroneHangarExerciseContext}
          />
          <View
            url={THEORY_URL.DroneHangar}
            exerciseId={
              <DroneHangarExerciseView context={DroneHangarExerciseContext} />
            }
          />
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}
