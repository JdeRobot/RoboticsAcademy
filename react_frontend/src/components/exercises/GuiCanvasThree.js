import { Box, Typography } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";

export default function GuiCanvasThree(props) {
  const { guiCanvasRef } = React.useContext(props.context);

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
      <Typography>Three JS Canvas </Typography>
      <canvas height={240} width={650} ref={guiCanvasRef} id="gui_canvas" />
    </Box>
  );
}
GuiCanvasThree.propTypes = {
  context: PropTypes.any,
};
