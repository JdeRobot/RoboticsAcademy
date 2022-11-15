import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/FollowRoadExerciseContext";
import MainAppBar from "../common/MainAppBar";
import FollowRoadExerciseContext from "../../contexts/FollowRoadExerciseContext";
import View from "../common/View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";
import FollowRoadExerciseView from "../views/FollowRoadExerciseView";

export default function FollowRoadReact() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <MainAppBar
            exerciseName={" Follow Road "}
            context={FollowRoadExerciseContext}
          />
          <View
            url={THEORY_URL.FollowRoad}
            exerciseId={
              <FollowRoadExerciseView context={FollowRoadExerciseContext} />
            }
          />
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}
