import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/DlDigitClassifierExerciseContext";
import ProminentAppBar from "../exercises/ProminentAppBar";
import DlDigitClassifierExerciseContext from "../../contexts/DlDigitClassifierExerciseContext";
import View from "../exercises/View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";
import DlDigitClassifierExerciseView from "../views/3DReconstructionExerciseView";

export default function DlDigitClassifierReact() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <ProminentAppBar
            exerciseName={" Digit Classifier "}
            context={DlDigitClassifierExerciseContext}
          />
          <View
            url={THEORY_URL.DlDigitClassifier}
            exerciseId={
              <DlDigitClassifierExerciseView
                context={DlDigitClassifierExerciseContext}
              />
            }
          />
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}
