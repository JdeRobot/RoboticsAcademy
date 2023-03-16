import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import NewManagerExerciseContext from "../../contexts/NewManagerExerciseContext";
import MainAppBar from "../common/MainAppBar";
import View from "../common/View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";
import RAMFollowLineExerciseView from "../views/RAM/RAMFollowLineExerciseView";
import CircuitSelector from "../buttons/RAM/RAMExerciseConfiguration";

function FollowLineReactRAM() {
  return (
    <Box>
      <ViewProvider>
        <>
          <MainAppBar
            exerciseName={" Follow Line RR"}
            specificConfiguration={<CircuitSelector></CircuitSelector>}
          />
          <View
            url={THEORY_URL.FollowLine}
            exerciseId={
              <RAMFollowLineExerciseView context={NewManagerExerciseContext} />
            }
          />
        </>
      </ViewProvider>
    </Box>
  );
}

export default FollowLineReactRAM;
