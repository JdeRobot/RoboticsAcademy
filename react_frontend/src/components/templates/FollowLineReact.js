import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/FollowLineExerciseContext";
import FollowLineExerciseContext from "../../contexts/FollowLineExerciseContext";
import ProminentAppBar from "../exercises/ProminentAppBar";
import View from "../exercises/View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";
import FollowLineExerciseView from "../views/FollowLineExerciseView";

function FollowLineReact() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <ProminentAppBar
            exerciseName={" Follow Line "}
            context={FollowLineExerciseContext}
          />
          <View
            url={THEORY_URL.FollowLine}
            exerciseId={
              <FollowLineExerciseView context={FollowLineExerciseContext} />
            }
          />
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}

export default FollowLineReact;
