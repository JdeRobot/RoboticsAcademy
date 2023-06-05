import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/DlDigitClassifierExerciseContext";
import MainAppBar from "../common/MainAppBar";
import DlDigitClassifierExerciseContext from "../../contexts/DlDigitClassifierExerciseContext";
import View from "../common/View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";
import DlDigitClassifierExerciseView from "../views/DlDigitClassifierExerciseView";

export default function DlDigitClassifierReact() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <MainAppBar
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
