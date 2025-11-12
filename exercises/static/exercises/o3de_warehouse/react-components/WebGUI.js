import { useState, useEffect, useRef} from "react";
import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import { updatePath, addToPath } from "./helpers/showImageVisual";
import RobotRed from "../resources/images/robot_red.svg";
import RobotGreen from "../resources/images/robot_green.svg";
import RobotBlue from "../resources/images/robot_blue.svg";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer from "Components/exercise/WebGUIContainer";

import warehouse from "../resources/images/map.png";

import "./css/GUICanvas.css"
function WebGUI(props) {
  const [realPose, setRealPose] = useState(null)
  const [noisyPose, setNoisyPose] = useState(null)
  const [realPath, setRealPath] = useState("")
  const [noisyPath, setNoisyPath] = useState("")
  const [mapImg, setMapImg] = useState(warehouse);
  const [userImage, setUserImage] = useState(undefined);
  const canvasRef = useRef(null);
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

      var img = canvasRef.current
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
        let image = JSON.parse(updateData.user_map);
        if (image.shape instanceof Array) {
          setUserImage(`data:image/png;base64,${image.user_map}`);
        }
      }

      // Send the ACK of the msg
      manager.send("gui", "ack");
    };

    const stateCallback = (message) => {
      if (message.data.state === "tools_ready") {
        setRealPose(null)
        setNoisyPose(null)
        setRealPath("")
        setNoisyPath("")
        realTrail=[]
        noisyTrail=[]
        setUserImage()
	      /*
        switch (manager.getUniverse()) {
          case "Laser Mapping Warehouse":
            setMapImg(warehouse);
            break;
          case "Small Laser Mapping Warehouse":
            setMapImg(smallWarehouse);
            break;
        }*/
	    setMapImg(warehouse);
      }
      valuesUntilValid = 0;
    }

    var img = canvasRef.current
    if (img) {
      resizeObserver.observe(img);
    }

    manager.subscribe(events.UPDATE, updateCallback);
    manager.subscribe(events.STATE_CHANGED, stateCallback);

    return () => {
      manager.unsubscribe(events.UPDATE, updateCallback);
      manager.unsubscribe(events.STATE_CHANGED, stateCallback);
    };
  }, [manager]);

  return (
    <WebGUIContainer>
      <WebGUIImage reference={canvasRef} id="map-img" src={mapImg} style={{ left: "0" }} />
      <WebGUIImage src={userImage} style={{ left: "50%" }} />
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
    </WebGUIContainer>
  );
}

export default WebGUI;
