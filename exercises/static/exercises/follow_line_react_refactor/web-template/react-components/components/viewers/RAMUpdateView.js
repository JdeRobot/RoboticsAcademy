import React, {Fragment} from "react";
import updateRenderer from "./UpdateRenderer";

import defaultCircuit from "../../images/default_circuit.png"
import "../../css/viewers/RAMUpdateView.css";

const RAMUpdateView = (props) => {
  const { width, height } = props;
  const canvasRef = React.useRef(null);
  const updateImageRef = React.useRef(new Image());
  const circuitImageRef = React.useRef(new Image());
  const carPositionRef = React.useRef([0, 0]);
  const rendererRef = React.useRef(new updateRenderer());
  const noImage = "https://via.placeholder.com/800x600.png?text=No%20image%20received%20from%20exercise";


  React.useEffect(() => {
    // Callback doesn't update renderer, it only stores the new image and position
    const callback = (message) => {
      const update = message.data.update;
      if (update.image) {
        const image = JSON.parse(update.image);
        updateImageRef.current.src = `data:image/png;base64,${image.image}`;
        carPositionRef.current = update.map.slice(1,).slice(0,-1).split(',');
      } else if(!update.brain) {
        updateImageRef.current.src = noImage;
      }
    };

    // Subscribe update event
    console.log("TestShowScreen subscribing to ['update'] events");
    RoboticsExerciseComponents.commsManager.subscribe([
        RoboticsExerciseComponents.commsManager.events.UPDATE
      ],
      callback);

    // Init renderer when circuit image is loaded
    circuitImageRef.current.src = defaultCircuit;
    circuitImageRef.current.onload = () => {
      rendererRef.current.init(width, height, canvasRef.current, updateImageRef, circuitImageRef, carPositionRef);
      rendererRef.current.run();
    }

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