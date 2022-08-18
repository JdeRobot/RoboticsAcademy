import { Box, Typography } from "@mui/material";
import * as React from "react";

export default function GuiCanvas() {
  return (
    <Box
      sx={{
        m: 3,
        p: 2,
        display: "flex",
        flexDirection: "column",
        border: "2px solid",
        alignItems: "center",
      }}
    >
      <Typography>Point of View</Typography>
      <img
        height={250}
        width={500}
        sx={{
          // marginX: 15,
          // marginY: 5,
          border: "2px solid #d3d3d3",
          backgroundRepeat: "no-repeat",
        }}
        id="gui_canvas"
      />
    </Box>
  );
}
