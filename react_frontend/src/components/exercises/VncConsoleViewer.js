import * as React from "react";
import Snackbar from "@mui/material/Snackbar";
import ExerciseContext from "../../contexts/ExerciseContext";
import { Box, Typography } from "@mui/material";
function VncConsoleViewer() {
  const { gazeboOn } = React.useContext(ExerciseContext);

  return (
    <Box>
      <Typography>Console</Typography>
      <iframe
        id={"console-vnc"}
        // scaleViewport
        // background="#000000"
        style={{
          width: "40vw",
          height: "50vh",
          display: "none",
        }}
      />
      <Snackbar
        open={gazeboOn}
        autoHideDuration={6000}
        message="Console Opened"
      />
    </Box>
  );
}

export default VncConsoleViewer;
