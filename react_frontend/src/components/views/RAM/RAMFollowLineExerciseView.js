import * as React from "react";
import { Box } from "@mui/material";
import RAMExerciseControl from "../../common/RAM/RAMExerciseControl";
import AceEditorRobot from "../../exercises/AceEditorRobot";
import RAMCircuitSelector from "../../visualizers/RAM/RAMCircuitSelector";
import GazeboViewer from "../../exercises/GazeboViewer";
import VncConsoleViewer from "../../exercises/VncConsoleViewer";
import VisualizationComponents from "../../common/VisualizationComponents";
import RAMImgCanvas from "../../visualizers/RAM/RAMImgCanvas";
import PropTypes from "prop-types";

import { LinterModal } from "../../modals/LInterModal";

function FollowLineExerciseView(props) {
  return (
    <Box id="exercise-view">
      <RAMExerciseControl context={props.context} />
      <Box
        sx={{
          display: "flex",

          alignItems: "center",
          flexDirection: "column",
          justifyContent: "space-around",
          p: 1,
          m: 1,
          background: "linear-gradient(#EOECDE, #FFFFFF)",
        }}
      >
        <AceEditorRobot context={props.context} />
        <div>
          <RAMCircuitSelector context={props.context} />
        </div>
        <VisualizationComponents>
          <RAMImgCanvas context={props.context} />
        </VisualizationComponents>
      </Box>
      <Box
        sx={{
          display: "flex",
          flexDirection: "column",
          border: "2px solid",
          p: 1,
          m: 1,
        }}
      >
        <Box
          sx={{
            display: "flex",
            alignItems: "center",
            justifyContent: "space-around",
            p: 2,
          }}
        >
          <GazeboViewer context={props.context} />
          <VncConsoleViewer context={props.context} />
        </Box>
      </Box>
      <LinterModal></LinterModal>
    </Box>
  );
}
FollowLineExerciseView.propTypes = {
  context: PropTypes.any,
};

export default FollowLineExerciseView;
