import * as React from "react";
import PropTypes from "prop-types";
import { drawImage, updatePath, addToPath } from "./helpers/showImageVisual";
import RobotRed from "../resources/images/robot_red.svg";
import RobotGreen from "../resources/images/robot_green.svg";
import RobotBlue from "../resources/images/robot_blue.svg";
import noImage from "../../assets/img/noImage.png";

import house from "../resources/images/map.png";

import "./css/GUICanvas.css"
function SpecificLaserMapping(props) {
  const [realPose, setRealPose] = React.useState(null)
  const [noisyPose, setNoisyPose] = React.useState(null)
  const [realPath, setRealPath] = React.useState("")
  const [noisyPath, setNoisyPath] = React.useState("")
  var realTrail = [];
  var noisyTrail = [];
  var realLastPose = undefined;
  var noisyLastPose = undefined;
  var valuesUntilValid = 0;

  const timeout = 40;

  const resizeObserver = new ResizeObserver((entries) => {
    var img = entries[0].target; 
    //or however you get a handle to the IMG
    var width = (img.clientWidth / 1500);
    var height = (img.clientHeight / 970);

    updatePath(realTrail, setRealPath, height, width);
    updatePath(noisyTrail, setNoisyPath, height, width);

    if (realLastPose) {
      setRealPose([realLastPose[0]*height,realLastPose[1]*width, 3.14 -realLastPose[2]]);
    }

    if (noisyLastPose) {
      setNoisyPose([noisyLastPose[0]*height,noisyLastPose[1]*width, 3.14 -noisyLastPose[2]]);
    }

    valuesUntilValid = 0;
  });

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");
    const callback = (message) => {
      const updateData = message.data.update;

      var img = document.getElementById('gui-canvas'); 
      //or however you get a handle to the IMG
      var width = (img.clientWidth / 1500);
      var height = (img.clientHeight / 970);

      if (updateData.real_pose) {
        const pose = updateData.real_pose.substring(1, updateData.real_pose.length - 1);
        const content = pose.split(",").map(item => parseFloat(item));
        realLastPose = content

        setRealPose([content[0]*height,content[1]*width, 3.14 -content[2]]);
        if (valuesUntilValid > timeout) {
          updatePath(realTrail, setRealPath, height, width);
          addToPath(content[0], content[1], realTrail);
        } else {
          valuesUntilValid = valuesUntilValid + 1;
        }
      }

      if (updateData.noisy_pose) {
        const pose = updateData.noisy_pose.substring(1, updateData.noisy_pose.length - 1);
        const content = pose.split(",").map(item => parseFloat(item));
        noisyLastPose = content

        setNoisyPose([content[0]*height,content[1]*width, 3.14 -content[2]]);
        if (valuesUntilValid > timeout) {
          updatePath(noisyTrail, setNoisyPath, height, width);
          addToPath(content[0], content[1], noisyTrail);
        }
      }

      if (updateData.user_map) {
        drawImage(updateData);
      }

      // Send the ACK of the msg
      window.RoboticsExerciseComponents.commsManager.send("gui", "ack");
    };

    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    resizeObserver.observe(document.getElementById('exercise-img'));

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
      if (message.data.state === "visualization_ready") {
        try {
          setRealPose(null)
          setNoisyPose(null)
          setRealPath("")
          setNoisyPath("")
          realTrail=[]
          noisyTrail=[]
        } catch (error) {
        }
      }
      valuesUntilValid = 0;
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
    <div style={{display: "flex", width: "100%", height: "100%", position:"relative"}}>
      <img src={house} alt="" className="exercise-canvas" id="exercise-img"/>
      <img className="image" id="gui-canvas" style={{left: "50%"}}
        src={noImage}/>
      {realPose &&
        <div id="real-pos" style={{rotate: "z "+ realPose[2]+"rad", top: realPose[0] -10 , left: realPose[1] -10}}>
          <img src={RobotGreen} id="real-pos"/>
        </div>
      }
      {realPath &&
        <svg height="100%" width="100%" xmlns="http://www.w3.org/2000/svg" style={{zIndex:2, position:"absolute"}}>
          <path xmlns="http://www.w3.org/2000/svg" d={realPath} 
            style={{strokeWidth: "1px", strokeLinejoin:"round", stroke: "green", fill: "none", opacity:"0.5"}}
          />
        </svg>
      }
      {noisyPose &&
        <div id="noisy-pos" style={{rotate: "z "+ noisyPose[2]+"rad", top: noisyPose[0] -10 , left: noisyPose[1] -10}}>
          <img src={RobotBlue} id="noisy-pos"/>
        </div>
      }
      {noisyPath &&
        <svg height="100%" width="100%" xmlns="http://www.w3.org/2000/svg" style={{zIndex:2, position:"absolute"}}>
          <path xmlns="http://www.w3.org/2000/svg" d={noisyPath} 
            style={{strokeWidth: "1px", strokeLinejoin:"round", stroke: "blue", fill: "none", opacity:"0.5"}}
          />
        </svg>
      }
    </div>
  );
}

SpecificLaserMapping.propTypes = {
  circuit: PropTypes.string,
};

export default SpecificLaserMapping;
