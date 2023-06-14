import * as React from "react";
import { Box } from "@mui/material";
import RAMExerciseControl from "../../common/RAM/RAMExerciseControl";
import AceEditorRobot from "../../exercises/AceEditorRobot";

import PropTypes from "prop-types";

import { LinterModal } from "../../modals/LInterModal";
import { Visualization } from "../Visualization";
import RAMImgCanvas from "../../visualizers/RAM/RAMImgCanvas";

function RealFollowPersonExerciseView(props) {
  return (
    <Box id="exercise-view">
      <RAMExerciseControl context={props.context} />
      <Box
        sx={{
          display: "flex",

          justifyContent: "space-around",
          p: 1,
          m: 1,
          background: "linear-gradient(#EOECDE, #FFFFFF)",
        }}
      >
        <AceEditorRobot context={props.context} />
        <Visualization
          context={props.context}
          specificVisualizator={<RAMImgCanvas />}
        ></Visualization>
      </Box>
      <LinterModal context={props.context}></LinterModal>
    </Box>
  );
}
RealFollowPersonExerciseView.propTypes = {
  context: PropTypes.any,
};

export default RealFollowPersonExerciseView;
