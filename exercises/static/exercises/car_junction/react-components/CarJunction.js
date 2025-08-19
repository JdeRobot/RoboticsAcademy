import * as React from "react";
import { Box } from "@mui/material";
import "./css/GUICanvas.css";

const CarJunction = (props) => {
  
  const [carPose, setCarPose] = React.useState(null)
  const canvasRef = React.useRef(null)

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");
    const callback = (message) => {
      if(message.data.update.image){
        const image = JSON.parse(message.data.update.image)
        if(image.image){
          let canvas = document.getElementById("canvas");
          //Parse encoded image data and decode it
          function decode_utf8(s) {
              return decodeURIComponent(escape(s))
          }
          var source = decode_utf8(image.image),
          shape = image.shape;

          if(source !== ""){
            canvas.src = "data:image/png;base64," + source;
            canvas.width = shape[1];
            canvas.height = shape[0];
          }
        }
        try {
          const pose = message.data.update.map
          setCarPose(pose)
        } catch (error) {
          
        }
      }
      
      window.RoboticsExerciseComponents.commsManager.send("gui", "ack");
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

  React.useEffect(() => {
    const callback = (message) => {
      console.log(message)
      if (message.data.state === "application_running") {
        window.RoboticsExerciseComponents.commsManager.send("gui", "start");
      } else if (message.data.state === "paused") {
        window.RoboticsExerciseComponents.commsManager.send("gui", "pause");
      } else if (message.data.state === "tools_ready") {
        setCarPose(null)      
      }
    }

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
  }, [])

  return (
    <Box sx={{ height: "100%", position: "relative"}}>
      <img ref={canvasRef} className={"exercise-canvas"} id="canvas"></img>
      {carPose && (
        <div className="overlay" style={{
          position: 'absolute',
          bottom: '20px',
          left: '20px',
          backgroundColor: 'rgba(0,0,0,0.7)',
          color: 'white',
          padding: '10px',
          borderRadius: '5px',
          fontFamily: 'monospace',
          fontSize: '16px'
        }}>
          Position: {carPose}
        </div>
      )}
    </Box>
  );
};

CarJunction.defaultProps = {
  width: 800,
  height: 600,
};

export default CarJunction