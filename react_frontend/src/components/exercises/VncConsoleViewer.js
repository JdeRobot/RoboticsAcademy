import * as React from "react";
import { Box, Typography } from "@mui/material";

function VncConsoleViewer() {
  return (
    <Box>
      <Typography>Console</Typography>
      <iframe
        id={"console-vnc"}
        // scaleViewport
        // background="#000000"
        style={{
          width: "75vw",
          height: "75vh",
        }}
      />
    </Box>
  );
}

export default VncConsoleViewer;
