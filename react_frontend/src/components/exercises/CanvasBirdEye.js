import * as React from "react";
import { Box, Typography } from "@mui/material";
import ExerciseContext from "../../contexts/ExerciseContext";
import "../../styles/birdsEyeCss.css";
export default function CanvasBirdEye() {
  const { birdEyeClass } = React.useContext(ExerciseContext);

  return (
    <Box
      sx={{
        m: 3,
        p: 2,
        display: "flex",
        flexDirection: "column",
        border: "2px solid",
        justifyContent: "space-around",
        alignItems: "center",
      }}
    >
      <Typography>Bird View</Typography>
      <canvas className={birdEyeClass} id="birds-eye" />
    </Box>
  );
}
