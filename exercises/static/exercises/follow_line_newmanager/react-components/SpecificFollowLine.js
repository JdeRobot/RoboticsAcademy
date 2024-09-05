import * as React from "react";
import { Box } from "@mui/material";
import "./css/GUICanvas.css";
import { drawImage } from "./helpers/showImagesFollowLine";
import { getProgress } from "./helpers/showProgressFollowLine";
import { getCarPose } from "./helpers/showCarPositionFollowLine";

import defaultCircuit from "../resources/images/default_circuit.png";

const SpecificFollowLine = (props) => {
  const [progress, setProgress] = React.useState(0)
  const [lapTime, setLapTime] = React.useState(null)
  const [carPose, setCarPose] = React.useState(null)
  const canvasRef = React.useRef(null)

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");
    const callback = (message) => {
      if(message.data.update.image){
        console.log(message.data.update)
        const image = JSON.parse(message.data.update.image)
        if(image.image){
          drawImage(message.data.update)
        }
        setLapTime(message.data.update.lap)
        setProgress(getProgress("simple", message.data.update.map))
        setCarPose(getCarPose(message.data.update.map))
      }
      
      // Send the ACK of the msg
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

  return (
    <Box sx={{ height: "100%", position: "relative"}}>
      <canvas
        ref={canvasRef}
        className={"exercise-canvas"}
        id="canvas"
      ></canvas>
      { lapTime &&
        <label className="overlay" id="lap-time">{lapTime} s</label>
      }
      <div className="overlay" id="progress-bar">
        <div id="progress-content" style={{width: progress+"%"}}/>
        <label id="progress-label">{progress}%</label>
      </div>
      <div className="overlay" id="circuit-aerial">
        <img src={defaultCircuit} alt="" id="circuit-img" />
        {carPose &&
          <div id="circuit-car-pos" style={{top: carPose[1], left: carPose[0]}}/>
        }
      </div>
    </Box>
  );
};

SpecificFollowLine.defaultProps = {
  width: 800,
  height: 600,
};

export default SpecificFollowLine
