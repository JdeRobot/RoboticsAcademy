import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/RoadJunctionExerciseContext";
import MainAppBar from "../common/MainAppBar";
import RoadJunctionExerciseContext from "../../contexts/RoadJunctionExerciseContext";
import View from "../common/View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";
import AutoParkingExerciseView from "../views/AutoParkingExerciseView";

function RoadJunctionReact() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <MainAppBar
            exerciseName={" Road Junction "}
            context={RoadJunctionExerciseContext}
          />
          <View
            url={THEORY_URL.RoadJunction}
            exerciseId={
              <AutoParkingExerciseView context={RoadJunctionExerciseContext} />
            }
          />
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}

export default RoadJunctionReact;
