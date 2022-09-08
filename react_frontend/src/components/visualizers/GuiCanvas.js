import { Box, Typography } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";
import "../../styles/GuiCanvas.css";

export default function GuiCanvas(props) {
  const {
    guiCanvasRefDrone,
    guiCanvasRef,
    exerciseSpecificCSS,
    canvasHeading,
  } = React.useContext(props.context);
  let guiCanvasRef_ = guiCanvasRef;
  let canvasId = "gui_canvas";
  if (props.drone) {
    guiCanvasRef_ = guiCanvasRefDrone;
    canvasId = "gui_canvas_left";
  }
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
      <Typography> {canvasHeading} </Typography>
      <canvas
        height={240}
        width={650}
        ref={guiCanvasRef_}
        className={exerciseSpecificCSS}
        id={canvasId}
      />
    </Box>
  );
}
GuiCanvas.propTypes = {
  context: PropTypes.any.isRequired,
  drone: PropTypes.bool,
};
