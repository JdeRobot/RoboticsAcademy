import * as React from "react";
import { Box } from "@mui/material";
import "./css/GUICanvas.css";
import { drawImage } from "./helpers/showImagesFollowLine";
import { getProgress, resetProgress } from "./helpers/showProgressFollowLine";
import { getCarPose } from "./helpers/showCarPositionFollowLine";
import { displayLapTime, isLapFinished, isInTimeoutLap, addLap } from "./helpers/showLapTimeFollowLine";

import defaultCircuit from "../resources/images/default_circuit.png";
import montmeloCircuit from "../resources/images/montmelo_circuit.png";
import montrealCircuit from "../resources/images/montreal_circuit.png";
import ngbCircuit from "../resources/images/ngb_circuit.png";

const width = 1280;
const height = 720;

const SpecificFollowLine = (props) => {
  const [progress, setProgress] = React.useState(0)
  const [lapTime, setLapTime] = React.useState(null)
  const [carPose, setCarPose] = React.useState(null)
  const [circuitImg, setCircuitImg] = React.useState(defaultCircuit);
  const canvasRef = React.useRef(null)
  var circuitName = "simple";
  var lapFinishedTime = null;
  var laps = [];

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");
    const callback = (message) => {
      if(message.data.update.image){
        console.log(message.data.update)
        const image = JSON.parse(message.data.update.image)
        if(image.image){
          drawImage(message.data.update)
        }
        try {
          const pose = getCarPose(circuitName, message.data.update.map)

          if (isLapFinished(circuitName, pose) && !isInTimeoutLap(lapFinishedTime, 6)) {
            lapFinishedTime = Date.now()
            laps.push(addLap(laps, message.data.update.lap))
            resetProgress()
            console.log(laps)
          }

          if (!isInTimeoutLap(lapFinishedTime, 6)) {
            setLapTime(displayLapTime(message.data.update.lap))
          }

          setProgress(getProgress(circuitName, pose))
          setCarPose(pose)
        } catch (error) {
          
        }
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

  React.useEffect(() => {
    const callback = (message) => {
      console.log(message)
      if (message.data.state === "application_running") {
        window.RoboticsExerciseComponents.commsManager.send("gui", "start");
      } else if (message.data.state === "paused") {
        window.RoboticsExerciseComponents.commsManager.send("gui", "pause");
      } else if (message.data.state === "visualization_ready") {
        lapFinishedTime = null;
        laps = [];
        resetProgress()
        setProgress(0)
        setCarPose(null)
        setLapTime(null)
        switch (context.mapSelected) {
          case "Default":
          case "follow_line_default_ros2":
            circuitName = "simple";
            setCircuitImg(defaultCircuit);
            break;
          case "Montmelo":
          case "follow_line_montmelo_ros2":
            circuitName = "montmelo";
            setCircuitImg(montmeloCircuit);
            break;
          case "Montreal":
          case "follow_line_montreal_ros2":
            circuitName = "montreal";
            setCircuitImg(montrealCircuit);
            break;
          case "Nürburgring":
          case "follow_line_nurburgring_ros2":
            circuitName = "nürburgring";
            setCircuitImg(ngbCircuit);
            break;
        }
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
        <img src={circuitImg} alt="" id="circuit-img" />
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
