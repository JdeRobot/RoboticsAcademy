import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/PowerTowerInspectionExerciseContext";
import MainAppBar from "../common/MainAppBar";
import PowerTowerInspectionExerciseContext from "../../contexts/PowerTowerInspectionExerciseContext";
import View from "../common/View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";
import PowerTowerInspectionExerciseView from "../views/PowerTowerInspectionExerciseView";

export default function PowerTowerInspectionReact() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <MainAppBar
            exerciseName={" Power Tower Inspection "}
            context={PowerTowerInspectionExerciseContext}
          />
          <View
            url={THEORY_URL.PowerTowerInspection}
            exerciseId={
              <PowerTowerInspectionExerciseView
                context={PowerTowerInspectionExerciseContext}
              />
            }
          />
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}
