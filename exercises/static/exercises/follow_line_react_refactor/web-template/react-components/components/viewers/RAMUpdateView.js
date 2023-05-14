import React from "react";
import updateRenderer from "./UpdateRenderer";

import defaultCircuit from "../../images/default_circuit.png"
import "../../css/viewers/RAMUpdateView.css";

const RAMUpdateView = (props) => {
  const { width, height } = props;
  const canvasRef = React.useRef(null);
  const updateRef = React.useRef({});
  const rendererRef = React.useRef(new updateRenderer());

  React.useEffect(() => {
    // Callback doesn't update renderer, it only stores the new image and position
    const callback = (message) => {
      updateRef.current = message.data.update;
    };

    // Subscribe update event
    console.log("TestShowScreen subscribing to ['update'] events");
    RoboticsExerciseComponents.commsManager.subscribe([
        RoboticsExerciseComponents.commsManager.events.UPDATE
      ],
      callback);

    // Init renderer when circuit image is loaded
    rendererRef.current.init(width, height, canvasRef.current, updateRef, defaultCircuit);
    rendererRef.current.run();

    // Unsubscribe update event and stop renderer
    return () => {
      console.log("TestShowScreen unsubscribing from ['state-changed'] events");
      RoboticsExerciseComponents.commsManager.unsubscribe([
          RoboticsExerciseComponents.commsManager.events.UPDATE
        ],
        callback);
      rendererRef.current.stop();
    }
  }, []);

  return (
    <canvas ref={canvasRef} className={"exercise-canvas"}></canvas>
  );
};

RAMUpdateView.defaultProps = {
  width: 800,
  height: 600
};

export default RAMUpdateView;