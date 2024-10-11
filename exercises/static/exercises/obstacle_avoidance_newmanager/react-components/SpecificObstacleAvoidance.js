import * as React from "react";
import PropTypes from "prop-types";

import F1Car from "../resources/images/f1-car.svg";
import Arrow from "../resources/images/arrow.svg";
import "./css/GUICanvas.css";

function SpecificObstacleAvoidance(props) {
  const meter = 73; // 1m = 73px

  const [laser, setLaser] = React.useState([])
  const [maxRange, setMaxRange] = React.useState([])
  const [carForce, setCarForce] = React.useState([2 * meter, 0])
  const [avgForce, setAvgForce] = React.useState([2 * meter, 0])
  const [obsForce, setObsForce] = React.useState([2 * meter, -Math.PI / 2])

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");

    const callback = (message) => {
      const data = message.data.update;
      if(data.map){
        const dataToDraw = JSON.parse(data.map)

        setLaser(dataToDraw.laser)
        setMaxRange(dataToDraw.max_range)
        var carForceDist = getDist(dataToDraw.car[0], dataToDraw.car[1])
        setCarForce([carForceDist * meter, getAng(dataToDraw.car[0], dataToDraw.car[1])])
        var avgForceDist = getDist(dataToDraw.average[0], dataToDraw.average[1])
        setAvgForce([avgForceDist * meter, getAng(dataToDraw.average[0], dataToDraw.average[1])])
        var obsForceDist = getDist(dataToDraw.obstacle[0], dataToDraw.obstacle[1])
        setObsForce([obsForceDist * meter, getAng(dataToDraw.obstacle[0], dataToDraw.obstacle[1])])
      }

      // Send the ACK of the msg
      window.RoboticsExerciseComponents.commsManager.send("gui", "ack");
    };

    const getDist = (x,y) => {
      return Math.sqrt(Math.pow(x,2) + Math.pow(y,2))
    }

    const getAng = (x,y) => {
      var ang = Math.PI / 2;
    	if(x != 0){
        ang = Math.atan2(y, x);
      }
      return ang;
    }

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
    height: "100%", backgroundColor: "#363233", position:"relative", overflow:"hidden"}}>
      <img src={F1Car} id="f1-car"/>
      {laser.map(element => {
        var ang = -element[1]
        var length = (element[0] / maxRange)*100;
        return (
          <hr className="laser-beam" style={{rotate: "z "+ ang +"rad", width: "calc("+length + "%)"}}/>
        )})
      }
      <img className="arrow green" src={Arrow} style={{height: carForce[0], rotate: "z "+ -carForce[1] +"rad"}}/>
      <img className="arrow red" src={Arrow} style={{height: obsForce[0], rotate: "z "+ -obsForce[1] +"rad"}}/>
      <img className="arrow" src={Arrow} style={{height: avgForce[0], rotate: "z "+ -avgForce[1] +"rad", zIndex: "6"}}/>
    </div>
  );
}

SpecificObstacleAvoidance.propTypes = {
  circuit: PropTypes.string,
};

export default SpecificObstacleAvoidance
