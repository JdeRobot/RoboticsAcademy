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
      updateRef.current = message.data.update;
    };

    console.log("TestShowScreen subscribing to ['update'] events");
    RoboticsExerciseComponents.commsManager.subscribe(
      [RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    return () => {
      console.log("TestShowScreen unsubscribing from ['update'] events");
      RoboticsExerciseComponents.commsManager.unsubscribe(
        [RoboticsExerciseComponents.commsManager.events.UPDATE],
        callback
      );
      rendererRef.current.stop();
    };
  }, []);

  React.useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "running") {
        rendererRef.current.run();
      } else if (message.data.state === "ready") {
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
        rendererRef.current.init(
          width,
          height,
          canvasRef.current,
          updateRef,
          circuit,
          context.mapSelected
        );
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
      console.log("TestShowScreen unsubscribing from ['state-changed'] events");
      RoboticsExerciseComponents.commsManager.unsubscribe(
        [RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
        callback
      );
    };
  }, [circuit]);

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
