import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/RescuePeopleExerciseContext";
import ProminentAppBar from "../exercises/ProminentAppBar";
import RescuePeopleExerciseContext from "../../contexts/RescuePeopleExerciseContext";
import View from "../exercises/View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";
import RescuePeopleExerciseView from "../views/RescuePeopleExerciseView";

export default function RescuePeopleReact() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <ProminentAppBar
            exerciseName={" Rescue People "}
            context={RescuePeopleExerciseContext}
          />
          <View
            url={THEORY_URL.RescuePeople}
            exerciseId={
              <RescuePeopleExerciseView context={RescuePeopleExerciseContext} />
            }
          />
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}
