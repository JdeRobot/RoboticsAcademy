import * as React from "react";
import { Box } from "@mui/material";
import RAMExerciseControl from "../../common/RAM/RAMExerciseControl";
import AceEditorRobot from "../../exercises/AceEditorRobot";

import PropTypes from "prop-types";

import { LinterModal } from "../../modals/LInterModal";
import { Visualization } from "../Visualization";
import { Frequencies } from "../../visualizers/RAM/RAMFrequency";
import SpecificVacuumCleaner from "../../visualizers/RAM/RAMSpecificVacuumCleaner";

function VacuumCleanerExerciseView(props) {
  return (
    <Box id="exercise-view">
      <RAMExerciseControl context={props.context} />
      <Box
        sx={{
          display: "flex",
          alignItems: "strech",
          height: "100%",
          justifyContent: "space-around",
          p: 1,
          m: 1,
          background: "linear-gradient(#EOECDE, #FFFFFF)",
        }}
      >
        <AceEditorRobot context={props.context} />
        <Visualization
          context={props.context}
          specificVisualizator={<SpecificVacuumCleaner />}
        ></Visualization>
      </Box>
      <Frequencies></Frequencies>
      <LinterModal context={props.context}></LinterModal>
    </Box>
  );
}
VacuumCleanerExerciseView.propTypes = {
  context: PropTypes.any,
};

export default VacuumCleanerExerciseView;
