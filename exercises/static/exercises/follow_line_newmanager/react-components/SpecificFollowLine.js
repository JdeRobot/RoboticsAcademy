import * as React from "react";
import { Box } from "@mui/material";
import "./css/GUICanvas.css";
import { drawImage } from "./helpers/showImagesFollowLine";
import updateRenderer from "./helpers/UpdateRenderer";
import defaultCircuit from "../resources/default_circuit.png";

const SpecificFollowLine = (props) => {
  const [image, setImage] = React.useState(null)
  const canvasRef = React.useRef(null)
  const updateRef = React.useRef({});
  const rendererRef = React.useRef(new updateRenderer());

  React.useEffect(() => {
    const callback = (message) => {
      console.log("Apoorv");
      console.log(message);
      if(message.data.update.image){
        console.log('image')
        const image = JSON.parse(message.data.update.image)
        if(image.image){
          updateRef.current = message.data.update;
          drawImage(message.data.update)
        } 
      }
    };

    console.log("TestShowScreen subscribing to ['update'] events");
    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    return () => {
      console.log("TestShowScreen unsubscribing from ['update'] events");
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
        callback
      );
      rendererRef.current.stop();
    };
  }, []);

  React.useEffect(() => {
    const callback = (message) => {
      console.log("Apoorv");
      console.log(message);
      if (message.data.state === "visualization_ready") {
        try {
          rendererRef.current.stop();
        } catch (error) {}

        rendererRef.current.init(
          800,
          600,
          canvasRef.current,
          updateRef,
          defaultCircuit,
          "simple"
        );
      }
      // if(message.data.update.image){
      //   console.log('image')
      //   const image = JSON.parse(message.data.update.image)
      //   console.log(image);
      //   // if(image.image){
      //   //   drawImage(message.data.update)
      //   // } 
      // }
    };

    console.log("TestShowScreen subscribing ['state-changed'] events");
    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
      callback
    );
    return () => {
      console.log("TestShowScreen unsubscribing from ['state-changed'] events");
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
        callback
      );
    };
  }, []);

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

SpecificFollowLine.defaultProps = {
  width: 800,
  height: 600,
};

export default SpecificFollowLine
