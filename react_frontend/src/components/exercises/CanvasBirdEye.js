import * as React from "react";
import { Box, Typography } from "@mui/material";
import ExerciseContext from "../../contexts/ExerciseContext";
export default function CanvasBirdEye() {
  const { backgroundImage, scaleToFit } = React.useContext(ExerciseContext);
  const BirdsEye = React.useRef(null);
  React.useEffect(() => {
    // Set Background Image for the first time
    let mapCanvas = BirdsEye.current;

    let ctx = mapCanvas.getContext("2d");
    ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
    let background = new Image();
    background.src = backgroundImage;
    // Make sure the image is loaded first otherwise nothing will draw.
    background.onload = function () {
      scaleToFit(background, ctx, mapCanvas);
    };
  }, []);
  return (
    <Box
      sx={{
        m: 3,
        p: 2,
        display: "inline-flex",
        flexDirection: "column",
        border: "2px solid #d3d3d3",
        maxHeight: 800,
        overflow: "auto",
      }}
    >
      <Typography align={"center"}>Bird View</Typography>
      <canvas
        id="birds-eye"
        ref={BirdsEye}
        height={500}
        width={500}
        sx={{
          marginX: 10,
          border: "2px solid #d3d3d3",
          backgroundRepeat: "no-repeat",
        }}
      />
      <Typography align={"center"}>Point of View</Typography>
      <img
        height={250}
        width={500}
        sx={{
          marginX: 15,
          marginY: 5,
          border: "2px solid #d3d3d3",
          backgroundRepeat: "no-repeat",
        }}
        id="gui_canvas"
      />
    </Box>
  );
}
