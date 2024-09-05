import React, { useEffect, useRef, useCallback } from "react";
import updateRenderer from "../helpers/UpdateRenderer";
import defaultCircuit from "../resources/images/default_circuit.png";
import montmeloCircuit from "../resources/images/montmelo_circuit.png";
import montrealCircuit from "../resources/images/montreal_circuit.png";
import ngbCircuit from "../resources/images/ngb_circuit.png";
import "../css/visualizers/UpdateView.css";
import { Box } from "@mui/material";

const UpdateView = (props) => {
  const width = 1280;
  const height = 720;

  const canvasRef = useRef(null);
  const updateRef = useRef({});
  const rendererRef = useRef(new updateRenderer());
  const [circuit, setCircuit] = React.useState(defaultCircuit);

  const updateRendererInit = useCallback(() => {
    rendererRef.current.init(
      width,
      height,
      canvasRef.current,
      updateRef,
      circuit,
      "simple"
    );
  }, [width, height, circuit]);

  useEffect(() => {
    const callback = (message) => {
      console.log('Update received:', message);
      updateRef.current = message.data.update;
      rendererRef.current.drawImage();
    };

    console.log("Subscribing to ['update'] events");
    RoboticsExerciseComponents.commsManager.subscribe(
      [RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    return () => {
      console.log("Unsubscribing from ['update'] events");
      RoboticsExerciseComponents.commsManager.unsubscribe(
        [RoboticsExerciseComponents.commsManager.events.UPDATE],
        callback
      );
      rendererRef.current.stop();
    };
  }, []);

  useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "application_running") {
        rendererRef.current.run();
      } else if (message.data.state === "visualization_ready") {
        switch (context.mapSelected) {
          case "Default":
          case "Default Ackermann":
            setCircuit(defaultCircuit);
            break;
          case "Montmelo":
          case "Montmelo Ackermann":
            setCircuit(montmeloCircuit);
            break;
          case "Montreal":
          case "Montreal Ackermann":
            setCircuit(montrealCircuit);
            break;
          case "Nürburgring":
          case "Nurburgring Ackermann":
            setCircuit(ngbCircuit);
            break;
          default:
            setCircuit(defaultCircuit);
        }
        try {
          rendererRef.current.stop();
        } catch (error) {}
        updateRendererInit();
      } else {
        try {
          rendererRef.current.stop();
        } catch (error) {}
      }
    };

    RoboticsExerciseComponents.commsManager.subscribe(
      [RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
      callback
    );

    return () => {
      console.log("Unsubscribing from ['state-changed'] events");
      RoboticsExerciseComponents.commsManager.unsubscribe(
        [RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
        callback
      );
    };
  }, [updateRendererInit]);

  return (
    <Box sx={{ height: "100%" }}>
      <canvas
        ref={canvasRef}
        className={"exercise-canvas"}
        id="canvas"
      ></canvas>
    </Box>
  );
};

UpdateView.defaultProps = {
  width: 800,
  height: 600,
};

export default UpdateView;
