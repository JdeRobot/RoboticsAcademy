import React, {Fragment} from "react";
import updateRenderer from "./UpdateRenderer";

import defaultCircuit from "../../images/default_circuit.jpg"
import "../../css/viewers/RAMUpdateView.css";

const RAMUpdateView = (props) => {
  const canvasRef = React.useRef(null);
  const requestIdRef = React.useRef(null);
  const updateImageRef = React.useRef(new Image());
  const circuitImage = React.useRef(new Image());
  const noImage = "https://via.placeholder.com/800x600.png?text=No%20image%20received%20from%20exercise";

  React.useEffect(() => {
    const callback = (message) => {
      const update = message.data.update;
      if (update.image) {
        const image = JSON.parse(update.image);
        updateImageRef.current.src = `data:image/png;base64,${image.image}`;
      } else if(!update.brain) {
        updateImageRef.current.src = noImage;
      }
    };

    console.log("TestShowScreen subscribing to ['update'] events");
    RoboticsExerciseComponents.commsManager.subscribe([
        RoboticsExerciseComponents.commsManager.events.UPDATE
      ],
      callback);

    circuitImage.current.src = defaultCircuit;
    initCanvas();
    requestIdRef.current = requestAnimationFrame(tickCanvas);

    return () => {
      console.log("TestShowScreen unsubscribing from ['state-changed'] events");

      RoboticsExerciseComponents.commsManager.unsubscribe([
          RoboticsExerciseComponents.commsManager.events.UPDATE
        ],
        callback);

      cancelAnimationFrame(requestIdRef.current);
    }
  }, []);

  const tickCanvas = () => {
    if(!canvasRef.current) return;
    updateCanvas();
    requestIdRef.current = requestAnimationFrame(tickCanvas);
  };

  const initCanvas = () => {
    canvasRef.current.width = canvasRef.current.offsetWidth;
    canvasRef.current.height = canvasRef.current.offsetHeight;
    updateImageRef.current.src = circuitImage.current.src;
  };

  const updateCanvas = () => {
    const ctx = canvasRef.current.getContext("2d");
    const image = updateImageRef.current;
    updateRenderer.call(ctx, image);
  }

  return (
    <canvas ref={canvasRef} className={"exercise-canvas"}></canvas>
  );
};

export default RAMUpdateView;