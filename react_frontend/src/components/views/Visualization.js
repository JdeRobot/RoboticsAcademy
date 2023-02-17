import { useContext } from "react";
import RAMImgCanvas from "../visualizers/RAM/RAMImgCanvas";
import React from "react";
import GazeboViewer from "../exercises/GazeboViewer";
import VncConsoleViewer from "../exercises/VncConsoleViewer";
import PropTypes from "prop-types";

export const Visualization = (props) => {
  const { visualization } = useContext(props.context);

  return (
    <>
      {visualization.specific ? <RAMImgCanvas context={props.context} /> : ""}
      {visualization.gazebo ? <GazeboViewer></GazeboViewer> : ""}
      {visualization.console ? <VncConsoleViewer></VncConsoleViewer> : ""}
    </>
  );
};

Visualization.propTypes = {
  context: PropTypes.any,
};
