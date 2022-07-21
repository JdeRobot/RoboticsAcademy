import * as React from "react";
import { Box, Typography } from "@mui/material";

function GazeboViewer() {
  return (
    <Box>
      <Typography>Gazebo</Typography>
      <iframe
        id={"iframe"}
        style={{
          width: "75vw",
          height: "75vh",
        }}
      />
    </Box>
  );
}

export default GazeboViewer;
