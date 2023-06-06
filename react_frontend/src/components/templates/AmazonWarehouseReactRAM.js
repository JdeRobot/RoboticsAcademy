import * as React from "react";
import { Box } from "@mui/material";
import { ViewProvider } from "../../contexts/ViewContext";
import { ExerciseProvider } from "../../contexts/NewManagerExerciseContext";
import NewManagerExerciseContext from "../../contexts/NewManagerExerciseContext";
import MainAppBar from "../common/MainAppBar";
import View from "../common/View";
import { THEORY_URL } from "../../helpers/TheoryUrlGetter";
import RAMFollowLineExerciseView from "../views/RAM/RAMFollowLineExerciseView"; // same views as Follow Line
import CircuitSelector from "../buttons/RAM/RAMExerciseConfiguration";
import { Footer } from "../common/RAM/Footer";

function AmazonWarehouseReactRAM() {
  return (
    <Box>
      <ViewProvider>
        <ExerciseProvider>
          <MainAppBar
            exerciseName={"Amazon Warehouse"}
            specificConfiguration={<CircuitSelector></CircuitSelector>}
          />
          <View
            url={THEORY_URL.AmazonWarehouse} // Theory URL missing
            exerciseId={
              <RAMFollowLineExerciseView context={NewManagerExerciseContext} />
            }
          />
          <Footer></Footer>
        </ExerciseProvider>
      </ViewProvider>
    </Box>
  );
}

export default AmazonWarehouseReactRAM;
