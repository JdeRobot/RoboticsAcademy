import { Box, Typography } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";
import "../../styles/GuiCanvas.css";

export default function GuiCanvas(props) {
  const { guiCanvasRef, exerciseSpecificCSS } = React.useContext(props.context);

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
      <Typography> View </Typography>
      <canvas
        height={240}
        width={650}
        ref={guiCanvasRef}
        className={exerciseSpecificCSS}
        id="gui_canvas"
      />
    </Box>
  );
}
GuiCanvas.propTypes = {
  context: PropTypes.any,
};
