import * as React from "react";
import { Box } from "@mui/material";
import "./css/GUICanvas.css";
import { drawImage } from "./helpers/showImagesFollowLine";

const SpecificFollowLine = (props) => {
  const canvasRef = React.useRef(null);

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");

    const callback = (message) => {
      const updateData = message.data.update;

      if (updateData.image) {
        console.log('Received image data');

        // Verify data, progress now unavaible
        const imageData = {
          image: updateData.image,
          lapTime: updateData.lap,
          progress: updateData.progress
        };

        drawImage(imageData);
      }

      // ACK
      window.RoboticsExerciseComponents.commsManager.send("gui", "ack");
    };

    // Update
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

export default SpecificFollowLine;
