import * as React from "react";
import { Box } from "@mui/material";
import "./css/GUICanvas.css";
import { drawImage } from "./helpers/showImages";


const DisplayFeed = (props) => {
  const [image, setImage] = React.useState(null)
  const canvasRef = React.useRef(null)

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");
    const callback = (message) => {
      if(message.data.update.image){
        console.log('image')
        const image = JSON.parse(message.data.update.image)
        if(image.image){
          drawImage(message.data.update)
        } 
      }
    };

    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    return () => {
      console.log("TestShowScreen unsubscribing from ['state-changed'] events");
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

DisplayFeed.defaultProps = {
  width: 800,
  height: 600,
};

export default DisplayFeed
