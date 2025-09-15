import { useState, useEffect} from "react";
import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import { drawImage, updatePath, addToPath } from "./helpers/showImageVisual";
import RobotRed from "../resources/images/robot_red.svg";
import RobotGreen from "../resources/images/robot_green.svg";
import RobotBlue from "../resources/images/robot_blue.svg";
import noImage from "../../assets/img/noImage.png";

import warehouse from "../resources/images/map.png";
import smallWarehouse from "../resources/images/small_map.png";

import "./css/GUICanvas.css"
function WebGUI(props) {
  const [realPose, setRealPose] = useState(null)
  const [noisyPose, setNoisyPose] = useState(null)
  const [realPath, setRealPath] = useState("")
  const [noisyPath, setNoisyPath] = useState("")
  const [mapImg, setMapImg] = useState(smallWarehouse);
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

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

  useEffect(() => {
    if (manager === null) {
      return;
    }

    const updateCallback = (message) => {
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
      manager.send("gui", "ack");
    };

    const stateCallback = (message) => {
      if (message.data.state === "tools_ready") {
        try {
          setRealPose(null)
          setNoisyPose(null)
          setRealPath("")
          setNoisyPath("")
          realTrail=[]
          noisyTrail=[]
          var canvas = document.getElementById("gui-canvas");
          canvas.src = noImage;
        } catch (error) {
        }
        switch (context.mapSelected) {
          case "Laser Mapping Warehouse":
            setMapImg(warehouse);
            break;
          case "Small Laser Mapping Warehouse":
            setMapImg(smallWarehouse);
            break;
        }
      }
      valuesUntilValid = 0;
    }

    resizeObserver.observe(document.getElementById("exercise-img"));

    manager.subscribe(events.UPDATE, updateCallback);
    manager.subscribe(events.STATE_CHANGED, stateCallback);

    return () => {
      manager.unsubscribe(events.UPDATE, updateCallback);
      manager.unsubscribe(events.STATE_CHANGED, stateCallback);
    };
  }, [manager]);

  return (
    <div style={{display: "flex", width: "100%", height: "100%", position:"relative"}}>
      <img src={mapImg} alt="" className="exercise-canvas" id="exercise-img"/>
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

export default WebGUI;
