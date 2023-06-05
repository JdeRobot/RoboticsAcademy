import * as React from "react";
import { Box, Typography } from "@mui/material";
import "../../styles/birdsEye.css";
import PropTypes from "prop-types";

export default function CanvasBirdEye(props) {
  const { birdEyeClass, birdEyeCanvas } = React.useContext(props.context);

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
      <canvas className={birdEyeClass} ref={birdEyeCanvas} id="birds-eye" />
    </Box>
  );
}

CanvasBirdEye.propTypes = {
  context: PropTypes.any,
};
