import * as React from "react";
import PropTypes from "prop-types";
import { paintEvent } from "./helpers/map";

import F1Car from "../resources/images/f1-car.svg";
import "./css/GUICanvas.css";

function SpecificObstacleAvoidance(props) {
  const guiCanvasRef = React.useRef();
  const [laser, setLaser] = React.useState([])
  const [maxRange, setMaxRange] = React.useState([])

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");

    const callback = (message) => {
      const data = message.data.update;
      if(data.map){
        const dataToDraw = JSON.parse(data.map)
        // TODO: For the arrows use the same as the lasers

        paintEvent(dataToDraw.target, dataToDraw.car, dataToDraw.obstacle, dataToDraw.average, dataToDraw.laser, dataToDraw.max_range)
        setLaser(dataToDraw.laser)
        setMaxRange(dataToDraw.max_range)
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
    <div style={{display: "flex",   width: "100%",
    height: "100%", backgroundColor: "#363233", position:"relative", overflowY:"auto"}}>
      <canvas
        ref={guiCanvasRef}
        id="local-map"

        style={{
          backgroundColor: "#363233",
          marginTop: "5px",
          width: "50%",
          height: "100%",
          margin: "auto",
      }}
      />
      <img src={F1Car} id="f1-car"/>
      {laser.map(element => {
        var ang = -element[1]
        var length = (element[0] / maxRange)*100;
        return (
          <hr className="laser-beam" style={{rotate: "z "+ ang +"rad", width: "calc("+length + "%)"}}/>
        )})
      }
    </div>
  );
}

SpecificObstacleAvoidance.propTypes = {
  circuit: PropTypes.string,
};

export default SpecificObstacleAvoidance
