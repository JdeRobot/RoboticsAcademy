import React from "react";
import "./exercise-index.css";
import Exercise from "./components/Exercise";
import { createRoot } from "react-dom/client";
import "./libs/tools.js";

import AceEditorRobot from "./components/exercises/AceEditorRobot";
import GazeboViewer from "./components/exercises/GazeboViewer";
import ProminentAppBar from "./components/exercises/ProminentAppBar";
import VncConsoleViewer from "./components/exercises/VncConsoleViewer";
import CircuitSelector from "./components/exercises/CircuitSelector";
import CanvasBirdEye from "./components/exercises/CanvasBirdEye";
import FollowLineReact from "./components/exercises/FollowLineReact";

const container = document.getElementById("exercise");
const root = createRoot(container);

window.RoboticsExerciseComponents = (function () {
  const components = {
    AceEditorRobot: AceEditorRobot,
    GazeboViewer: GazeboViewer,
    ProminentAppBar: ProminentAppBar,
    VncConsoleViewer: VncConsoleViewer,
    CircuitSelector: CircuitSelector,
    CanvasBirdEye: CanvasBirdEye,
    FollowLineReact: FollowLineReact,
  };

  const render = function (component, container, props, children, callback) {
    if (typeof component == "string") component = components[component];

    const element = React.createElement(component, props, children, callback);
    const root = createRoot(container);
    root.render(element);
  };

  return {
    render: render,
    components: components,
  };
})();

root.render(
  <React.StrictMode>
    <Exercise></Exercise>
  </React.StrictMode>
);
