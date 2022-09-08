import * as React from "react";
import { Box, Typography } from "@mui/material";
import ExerciseContext from "../../contexts/ExerciseContext";

function GazeboViewer() {
  const { openGazebo } = React.useContext(ExerciseContext);

  return (
    <Box display={openGazebo ? "block" : "none"}>
      <Typography color={"secondary"} borderBottom={2} m={0.4}>
        Gazebo
      </Typography>
      <iframe
        id={"iframe"}
        style={{
          width: "40vw",
          height: "50vh",
        }}
        src={
          openGazebo
            ? "http://127.0.0.1:6080/vnc.html?resize=remote&autoconnect=true"
            : ""
        }
      />
    </Box>
  );
}

export default GazeboViewer;
