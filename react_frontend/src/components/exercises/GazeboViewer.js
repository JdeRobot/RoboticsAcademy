import * as React from "react";
import { Box, Typography } from "@mui/material";

function GazeboViewer() {
  return (
    <Box>
      <Typography>Gazebo</Typography>
      <iframe
        id={"iframe"}
        style={{
          width: "40vw",
          height: "50vh",
          display: "none",
        }}
      />
    </Box>
  );
}

export default GazeboViewer;
