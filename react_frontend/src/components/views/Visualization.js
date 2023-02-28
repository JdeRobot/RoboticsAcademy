import { useContext } from "react";
import React from "react";
import GazeboViewer from "../exercises/GazeboViewer";
import VncConsoleViewer from "../exercises/VncConsoleViewer";
import PropTypes from "prop-types";
import { Box } from "@mui/material";

export const Visualization = (props) => {
  const { visualization } = useContext(props.context);

  return (
    <Box
      sx={{
        m: 1,
        display: "flex",
        flexDirection: "column",
        width: "100%",
        alignItems: "center",
        justifyContent: "flex-start",
      }}
    >
      {visualization.specific ? props.specificVisualizator : ""}
      {visualization.gazebo ? <GazeboViewer></GazeboViewer> : ""}
      {visualization.console ? <VncConsoleViewer></VncConsoleViewer> : ""}
    </Box>
  );
};

Visualization.propTypes = {
  context: PropTypes.any,
  specificVisualizator: PropTypes.any,
};
