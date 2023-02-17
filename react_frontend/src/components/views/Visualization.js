import { useContext } from "react";
import RAMImgCanvas from "../visualizers/RAM/RAMImgCanvas";
import React from "react";
import GazeboViewer from "../exercises/GazeboViewer";
import VncConsoleViewer from "../exercises/VncConsoleViewer";
import PropTypes from "prop-types";

export const Visualization = (props) => {
  const { visualization } = useContext(props.context);

  if (visualization === "camera") {
    return <RAMImgCanvas context={props.context} />;
  }
  if (visualization === "gazebo") {
    return <GazeboViewer></GazeboViewer>;
  }
  if (visualization === "console") {
    return <VncConsoleViewer></VncConsoleViewer>;
  }
  return "";
};

Visualization.propTypes = {
  context: PropTypes.any,
};
