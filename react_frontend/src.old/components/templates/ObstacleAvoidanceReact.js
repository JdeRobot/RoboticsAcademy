import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/ObstacleAvoidanceExerciseContext";
import MainAppBar from "../common/MainAppBar";
import ObstacleAvoidanceExerciseContext from "../../contexts/ObstacleAvoidanceExerciseContext";
import View from "../common/View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";
import ObstacleAvoidanceExerciseView from "../views/ObstacleAvoidanceExerciseView";

function ObstacleAvoidanceReact() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <MainAppBar
            exerciseName={" Obstacle Avoidance "}
            context={ObstacleAvoidanceExerciseContext}
          />
          <View
            url={THEORY_URL.ObstacleAvoidance}
            exerciseId={
              <ObstacleAvoidanceExerciseView
                context={ObstacleAvoidanceExerciseContext}
              />
            }
          />
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}

export default ObstacleAvoidanceReact;
