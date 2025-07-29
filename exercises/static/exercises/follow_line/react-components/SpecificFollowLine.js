import * as React from "react";
import { Box } from "@mui/material";
import "./css/GUICanvas.css";
import { getCarPose } from "./helpers/showCarPositionFollowLine";
import { displayLapTime } from "./helpers/showLapTimeFollowLine";

import defaultCircuit from "../resources/images/default_circuit.png";
import montmeloCircuit from "../resources/images/montmelo_circuit.png";
import montrealCircuit from "../resources/images/montreal_circuit.png";
import ngbCircuit from "../resources/images/ngb_circuit.png";

const SpecificFollowLine = (props) => {
  const [lapTime, setLapTime] = React.useState(null);
  const [carPose, setCarPose] = React.useState(null);
  const [circuitImg, setCircuitImg] = React.useState(defaultCircuit);
  const canvasRef = React.useRef(null);
  var circuitName = "simple";

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");
    const callback = (message) => {
      if (message.data.update.action === "stop") {
        window.RoboticsExerciseComponents.commsManager
          .pause()
          .then(() => {})
          .catch((response) => console.log(response));
        return;
      }

      if (message.data.update.image) {
        const image = JSON.parse(message.data.update.image);
        if (image.image) {
          let canvas = document.getElementById("canvas");
          //Parse encoded image data and decode it
          function decode_utf8(s) {
            return decodeURIComponent(escape(s));
          }
          var source = decode_utf8(image.image),
            shape = image.shape;

          if (source !== "") {
            canvas.src = "data:image/png;base64," + source;
            canvas.width = shape[1];
            canvas.height = shape[0];
          }
        }
        try {
          const pose = getCarPose(circuitName, message.data.update.map);
          setLapTime(displayLapTime(message.data.update.lap));
          setCarPose(pose);
        } catch (error) {}
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
      console.log(message);
      if (message.data.state === "application_running") {
        window.RoboticsExerciseComponents.commsManager.send("gui", "start");
      } else if (message.data.state === "paused") {
        window.RoboticsExerciseComponents.commsManager.send("gui", "pause");
      } else if (message.data.state === "tools_ready") {
        setCarPose(null);
        setLapTime(null);
        switch (context.mapSelected) {
          case "Simple Circuit":
            circuitName = "default";
            setCircuitImg(defaultCircuit);
            break;
          case "Montmelo Circuit":
            circuitName = "montmelo";
            setCircuitImg(montmeloCircuit);
            break;
          case "Montreal Circuit":
            circuitName = "montreal";
            setCircuitImg(montrealCircuit);
            break;
          case "Nurburgring Circuit":
            circuitName = "ngb";
            setCircuitImg(ngbCircuit);
            break;
          case "Simple Ackermann Circuit":
            circuitName = "default ack";
            setCircuitImg(defaultCircuit);
            break;
          case "Montmelo Ackermann Circuit":
            circuitName = "montmelo ack";
            setCircuitImg(montmeloCircuit);
            break;
          case "Montreal Ackermann Circuit":
            circuitName = "montreal ack";
            setCircuitImg(montrealCircuit);
            break;
          case "Nurburgring Ackermann Circuit":
            circuitName = "ngb ack";
            setCircuitImg(ngbCircuit);
            break;
        }
      }
    };

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
    <Box sx={{ height: "100%", position: "relative" }}>
      <img ref={canvasRef} className={"exercise-canvas"} id="canvas"></img>
      {lapTime && (
        <label className="overlay" id="lap-time">
          {lapTime} s
        </label>
      )}
      <div className="overlay" id="circuit-aerial">
        <img src={circuitImg} alt="" id="circuit-img" />
        {carPose && (
          <div
            id="circuit-car-pos"
            style={{ top: carPose[1], left: carPose[0] }}
          />
        )}
      </div>
    </Box>
  );
};

SpecificFollowLine.defaultProps = {
  width: 800,
  height: 600,
};

export default SpecificFollowLine;
