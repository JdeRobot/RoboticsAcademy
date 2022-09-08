import * as React from "react";
import { Box } from "@mui/material";
import PropTypes from "prop-types";

export default function VisualizationComponents(props) {
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
      {props.children}
    </Box>
  );
}

VisualizationComponents.propTypes = {
  children: PropTypes.node,
};
