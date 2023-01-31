import * as React from "react";
import { Box, Typography } from "@mui/material";
import RAMExerciseControl from "../../common/RAM/RAMExerciseControl";
import AceEditorRobot from "../../exercises/AceEditorRobot";
import RAMCircuitSelector from "../../visualizers/RAM/RAMCircuitSelector";
import GazeboViewer from "../../exercises/GazeboViewer";
import VncConsoleViewer from "../../exercises/VncConsoleViewer";
import VisualizationComponents from "../../common/VisualizationComponents";
import FrequencyMenu from "../../common/FrequencyMenu";
import RAMImgCanvas from "../../visualizers/RAM/RAMImgCanvas";
import PropTypes from "prop-types";

import { LinterModal } from "../../modals/LInterModal";

function FollowLineExerciseView(props) {
  return (
    <Box id="exercise-view">
      <RAMExerciseControl context={props.context} teleOpMode={true} />
      <Box
        sx={{
          display: "flex",
          border: "2px solid",
          alignItems: "center",
          flexDirection: "column",
          justifyContent: "space-around",
          p: 1,
          m: 1,
          background: "linear-gradient(#EOECDE, #FFFFFF)",
        }}
      >
        <Typography align={"center"} color={"primary"} variant={"h4"}>
          {" "}
          Start Coding !{" "}
        </Typography>
        <AceEditorRobot context={props.context} />
        <Typography align={"center"} m={2} color={"primary"} variant={"h4"}>
          Visualization
        </Typography>
        <div>
          <RAMCircuitSelector context={props.context} />
        </div>
        <VisualizationComponents>
          <RAMImgCanvas context={props.context} />
          <FrequencyMenu context={props.context} />
        </VisualizationComponents>
      </Box>
      {/*  </Box>*/}
      {/*</Box>*/}
      <Box
        sx={{
          display: "flex",
          flexDirection: "column",
          border: "2px solid",
          p: 1,
          m: 1,
        }}
      >
        <Typography align={"center"} m={2} color={"primary"} variant={"h4"}>
          {" "}
          Simulation and Console !{" "}
        </Typography>
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
          <button
            onClick={() => {
              window.RoboticsExerciseComponents.commsManager
                .pause()
                .then(() => {
                  console.log("paused");
                })
                .catch((response) => console.log(response));
            }}
          >
            pause
          </button>
          <button
            onClick={() => {
              window.RoboticsExerciseComponents.commsManager
                .run()
                .then(() => {
                  console.log("running");
                })
                .catch((response) => console.log(response));
            }}
          >
            play
          </button>
          <button
            onClick={() => {
              window.RoboticsExerciseComponents.commsManager
                .resume()
                .then(() => {
                  console.log("running");
                })
                .catch((response) => console.log(response));
            }}
          >
            resume
          </button>
        </Box>
      </Box>
      {/*       <LoadModal context={props.context} />
      <CustomAlert context={props.context} />
      <ErrorModal context={props.context} />
      <InfoModal context={props.context} /> */}
      <LinterModal></LinterModal>
    </Box>
  );
}
FollowLineExerciseView.propTypes = {
  context: PropTypes.any,
};

export default FollowLineExerciseView;
