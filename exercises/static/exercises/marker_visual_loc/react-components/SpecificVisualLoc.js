import * as React from "react";
import PropTypes from "prop-types";
import { drawImage, updatePath, addToPath } from "./helpers/showImageVisual";
import RobotRed from "../resources/images/robot_red.svg";
import RobotGreen from "../resources/images/robot_green.svg";
import RobotBlue from "../resources/images/robot_blue.svg";
import noImage from "../../assets/img/noImage.png";

import house from "../resources/images/map.png";

import "./css/GUICanvas.css";
function SpecificVisualLoc(props) {
  const [realPose, setRealPose] = React.useState(null)
  const [noisyPose, setNoisyPose] = React.useState(null)
  const [userPose, setUserPose] = React.useState(null)
  const [realPath, setRealPath] = React.useState("")
  const [noisyPath, setNoisyPath] = React.useState("")
  const [userPath, setUserPath] = React.useState("")
  const [resizedBeacons, setResizedBeacons] = React.useState({})
  var realTrail = [];
  var noisyTrail = [];
  var userTrail = [];
  var realLastPose = undefined;
  var noisyLastPose = undefined;
  var userLastPose = undefined;
  var valuesUntilValid = 0;

  const timeout = 40;

  const beacons = [
    { id: "tag_0", x: 518.75, y: 270.325, type: "hor" },
    { id: "tag_1", x: 481.4, y: 810.775, type: "hor" },
    { id: "tag_2", x: 196.395, y: 339.15, type: "vert" },
    { id: "tag_3", x: 400.89, y: 79.9, type: "hor" },
    { id: "tag_4", x: 844.94, y: 712.3, type: "vert" },
    { id: "tag_5", x: 295.03, y: 499.8, type: "vert" },
    { id: "tag_6", x: 730.4, y: 350.55, type: "hor" },
    { id: "tag_7", x: 499.66, y: 140.25, type: "vert" },
  ];

  const resizeObserver = new ResizeObserver((entries) => {
    var img = entries[0].target; 
    //or however you get a handle to the IMG
    var width = (img.clientWidth / 1012);
    var height = (img.clientHeight / 1012);

    setResizedBeacons(beacons.map(beacon => ({
      id: beacon.id,
      x: beacon.x * width,
      y: beacon.y * height,
      type: beacon.type,
    })))

    updatePath(realTrail, setRealPath, height, width);
    updatePath(noisyTrail, setNoisyPath, height, width);
    updatePath(userTrail, setUserPath, height, width);

    if (realLastPose) {
      setRealPose([realLastPose[1]*height,realLastPose[0]*width, -1.57 -realLastPose[2]]);
    }

    if (noisyLastPose) {
      setNoisyPose([noisyLastPose[1]*height,noisyLastPose[0]*width, -1.57 -noisyLastPose[2]]);
    }

    if (userLastPose) {
      setUserPose([userLastPose[1]*height,userLastPose[0]*width, -1.57 -userLastPose[2]]);
    }

    valuesUntilValid = 0;
  });

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");
    const callback = (message) => {
      const updateData = message.data.update;

      var img = document.getElementById('gui-canvas'); 
      //or however you get a handle to the IMG
      var width = (img.clientWidth / 1012);
      var height = (img.clientHeight / 1012);

      if (updateData.real_pose) {
        const pose = updateData.real_pose.substring(1, updateData.real_pose.length - 1);
        const content = pose.split(",").map(item => parseFloat(item));
        realLastPose = content

        setRealPose([content[1]*height,content[0]*width, -1.57 -content[2]]);
        if (valuesUntilValid > timeout) {
          updatePath(realTrail, setRealPath, height, width);
          addToPath(content[1], content[0], realTrail);
        } else {
          valuesUntilValid = valuesUntilValid + 1;
        }
      }

      if (updateData.noisy_pose) {
        const pose = updateData.noisy_pose.substring(1, updateData.noisy_pose.length - 1);
        const content = pose.split(",").map(item => parseFloat(item));
        noisyLastPose = content

        setNoisyPose([content[1]*height,content[0]*width, -1.57 -content[2]]);
        if (valuesUntilValid > timeout) {
          updatePath(noisyTrail, setNoisyPath, height, width);
          addToPath(content[1], content[0], noisyTrail);
        }
      }

      if (updateData.estimate_pose) {
        const pose = updateData.estimate_pose.substring(1, updateData.estimate_pose.length - 1);
        const content = pose.split(",").map(item => parseFloat(item));
        userLastPose = content

        setUserPose([content[1]*height,content[0]*width, -1.57 -content[2]]);
        if (valuesUntilValid > timeout) {
          updatePath(userTrail, setUserPath, height, width);
          addToPath(content[1], content[0], userTrail);
        }
      }

      if (updateData.image) {
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
          setUserPose(null)
          setRealPath("")
          setNoisyPath("")
          setUserPath("")
          realTrail=[]
          noisyTrail=[]
          userTrail=[]

          var img = document.getElementById('gui-canvas'); 
          //or however you get a handle to the IMG
          var width = (img.clientWidth / 1012);
          var height = (img.clientHeight / 1012);
          setResizedBeacons(beacons.map(beacon => ({
            id: beacon.id,
            x: beacon.x * width,
            y: beacon.y * height,
            type: beacon.type,
          })))
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
        <div id="real-pos" style={{rotate: "z "+ realPose[2]+"rad", top: realPose[0] -10 , left: realPose[1] -5}}>
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
        <div id="noisy-pos" style={{rotate: "z "+ noisyPose[2]+"rad", top: noisyPose[0] -10 , left: noisyPose[1] -5}}>
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
      {userPose &&
        <div id="user-pos" style={{rotate: "z "+ userPose[2]+"rad", top: userPose[0] -10 , left: userPose[1] -5}}>
          <img src={RobotRed} id="user-pos"/>
        </div>
      }
      {userPath &&
        <svg height="100%" width="100%" xmlns="http://www.w3.org/2000/svg" style={{zIndex:2, position:"absolute"}}>
          <path xmlns="http://www.w3.org/2000/svg" d={userPath} 
            style={{strokeWidth: "1px", strokeLinejoin:"round", stroke: "red", fill: "none", opacity:"0.5"}}
          />
        </svg>
      }
      {Object.values(resizedBeacons).map((beacon) => {
        return(
        <div
          key={beacon.id}
          className={`beacon ${beacon.type}`}
          style={{
            top: `${beacon.y}px`,
            left: `${beacon.x}px`,
            position: "absolute",
            border: "2px solid rgb(255, 255, 255)",
            cursor: "pointer",
            zIndex: "5",
            width: `${(beacon.type == "vert") ? 0 : 20}px`,
            height: `${(beacon.type == "hor") ? 0 : 20}px`,
          }}
          title={`ID: ${beacon.id}`}
        />
      )})}
    </div>
  );
}

export default SpecificVisualLoc;