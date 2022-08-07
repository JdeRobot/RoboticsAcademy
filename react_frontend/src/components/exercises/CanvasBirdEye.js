import * as React from "react";
import { Box, Typography } from "@mui/material";
import ExerciseContext from "../../contexts/ExerciseContext";
import "../../styles/birdsEyeCss.css";
import FrequencyMenu from "./FrequencyMenu";
export default function CanvasBirdEye() {
  const { birdEyeClass } = React.useContext(ExerciseContext);

  return (
    <Box
      sx={{
        m: 3,
        p: 2,
        display: "inline-flex",
        width: "100%",
        alignItems: "stretch",
        justifyContent: "space-around",
      }}
    >
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
        <Typography>Frequency Menu</Typography>
        <FrequencyMenu />
      </Box>
    </Box>
  );
}
