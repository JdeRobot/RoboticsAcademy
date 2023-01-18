import * as React from "react";
import PropTypes from "prop-types";
import { Box, Typography } from "@mui/material";

export default function CanvasThree(props) {
  const { canvasRef } = React.useContext(props.context);

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
      <Typography> 3d Environment </Typography>
      <div ref={canvasRef}> </div>
    </Box>
  );
}
CanvasThree.propTypes = {
  context: PropTypes.any,
  heading: PropTypes.string,
};
