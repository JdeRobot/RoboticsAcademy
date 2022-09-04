import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/GlobalNavigationExerciseContext";
import GlobalNavigationExerciseContext from "../../contexts/GlobalNavigationExerciseContext";
import ProminentAppBar from "../exercises/ProminentAppBar";
import View from "../exercises/View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";
import GlobalNavigationExerciseView from "../views/GlobalNavigationExerciseView";

export default function GlobalNavigationReact() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <ProminentAppBar
            exerciseName={" Global Navigation "}
            context={GlobalNavigationExerciseContext}
          />
          <View
            url={THEORY_URL.GlobalNavigation}
            exerciseId={
              <GlobalNavigationExerciseView
                context={GlobalNavigationExerciseContext}
              />
            }
          />
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}
