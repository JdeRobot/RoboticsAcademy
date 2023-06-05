import { Box, Typography } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";

export default function ImgCanvas(props) {
  const { imageRef } = React.useContext(props.context);
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
      <Typography>{props.heading}</Typography>
      <img
        height={250}
        width={500}
        sx={{
          // marginX: 15,
          // marginY: 5,
          border: "2px solid #d3d3d3",
          backgroundRepeat: "no-repeat",
        }}
        ref={imageRef}
        id="gui_canvas"
      />
    </Box>
  );
}

ImgCanvas.propTypes = {
  context: PropTypes.any,
  heading: PropTypes.string,
};
