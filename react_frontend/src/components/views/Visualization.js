import { useContext } from "react";
import RAMImgCanvas from "../visualizers/RAM/RAMImgCanvas";
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
        display: "flex",
        width: "100%",
        flexWrap: "wrap",
        alignItems: "stretch",
        justifyContent: "space-around",
      }}
    >
      {visualization.specific ? <RAMImgCanvas context={props.context} /> : ""}
      {visualization.gazebo ? <GazeboViewer></GazeboViewer> : ""}
      {visualization.console ? <VncConsoleViewer></VncConsoleViewer> : ""}
    </Box>
  );
};

Visualization.propTypes = {
  context: PropTypes.any,
};
