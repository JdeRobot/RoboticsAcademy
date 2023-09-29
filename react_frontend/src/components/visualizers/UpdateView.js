import React from "react";
import updateRenderer from "../../libs/UpdateRenderer";
import defaultCircuit from "../../images/default_circuit.png";
import montmeloCircuit from "../../images/montmelo_circuit.png";
import montrealCircuit from "../../images/montreal_circuit.png";
import ngbCircuit from "../../images/ngb_circuit.png";
import "../../styles/visualizers/UpdateView.css";
import { Box } from "@mui/material";

const UpdateView = (props) => {
  const width = 1280;
  const height = 720;
  const canvasRef = React.useRef(null);
  const updateRef = React.useRef({});
  const rendererRef = React.useRef(new updateRenderer());
  const [circuit, setCircuit] = React.useState(defaultCircuit);

  React.useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "ready") {
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
      }
    };
    console.log("TestShowScreen subscribing to ['update'] events");
    RoboticsExerciseComponents.commsManager.subscribe(
      [RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
      callback
    );

    return () => {
      console.log("TestShowScreen unsubscribing from ['state-changed'] events");
      RoboticsExerciseComponents.commsManager.unsubscribe(
        [RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
        callback
      );
      rendererRef.current.stop();
    };
  }, []);

  React.useEffect(() => {
    // Callback doesn't update renderer, it only stores the new image and position
    const callback = (message) => {
      updateRef.current = message.data.update;
    };

    // Subscribe update event
    console.log("TestShowScreen subscribing to ['update'] events");
    RoboticsExerciseComponents.commsManager.subscribe(
      [RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    // Init renderer when circuit image is loaded
    rendererRef.current.init(
      width,
      height,
      canvasRef.current,
      updateRef,
      circuit
    );
    rendererRef.current.run();

    // Unsubscribe update event and stop renderer
    return () => {
      console.log("TestShowScreen unsubscribing from ['state-changed'] events");
      RoboticsExerciseComponents.commsManager.unsubscribe(
        [RoboticsExerciseComponents.commsManager.events.UPDATE],
        callback
      );
      rendererRef.current.stop();
    };
  }, [circuit]);

  return (
    <Box sx={{ height: "100%" }}>
      <canvas ref={canvasRef} className={"exercise-canvas"}></canvas>
    </Box>
  );
};

UpdateView.defaultProps = {
  width: 800,
  height: 600,
};

export default UpdateView;
