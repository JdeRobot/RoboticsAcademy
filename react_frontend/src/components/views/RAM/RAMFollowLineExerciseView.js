import * as React from "react";
import { Box } from "@mui/material";
import RAMExerciseControl from "../../common/RAM/RAMExerciseControl";
import AceEditorRobot from "../../exercises/AceEditorRobot";
import VisualizationComponents from "../../common/VisualizationComponents";

import PropTypes from "prop-types";

import { LinterModal } from "../../modals/LInterModal";
import { Visualization } from "../Visualization";

function FollowLineExerciseView(props) {
  return (
    <Box id="exercise-view">
      <RAMExerciseControl context={props.context} />
      <Box
        sx={{
          display: "flex",
          alignItems: "center",
          justifyContent: "space-around",
          p: 1,
          m: 1,
          background: "linear-gradient(#EOECDE, #FFFFFF)",
        }}
      >
        <AceEditorRobot context={props.context} />
        {/* <RAMCircuitSelector context={props.context} /> */}
        <VisualizationComponents>
          <Visualization context={props.context}></Visualization>
        </VisualizationComponents>
      </Box>

      <LinterModal></LinterModal>
    </Box>
  );
}
FollowLineExerciseView.propTypes = {
  context: PropTypes.any,
};

export default FollowLineExerciseView;
