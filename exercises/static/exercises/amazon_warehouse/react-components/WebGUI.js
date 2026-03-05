import { useState, useEffect} from "react";
import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import {updatePath, addToTrail, updateTrail} from "./helpers/AmazonWarehouseHelper";

import Map1 from "../resources/images/map.png"
import Map2 from "../resources/images/map_2.png"

import "./css/GUICanvas.css";

function WebGUI() {
  const Map1Size = {width: 415, height: 279}
  const Map2Size = {width: 1075, height: 699}

  const [map, setMap] = useState(Map1)
  const [mapSize, setMapSize] = useState(Map1Size)
  const [vehicleType, setVehicleType] = useState(0) // 0=normal 1=ackermann
  const [vehiclePose, setVehiclePose] = useState(null)
  const [targetPose, setTargetPose] = useState(null)
  const [trail, setTrail] = useState("")
  const [path, setPath] = useState("")
  const [liftState, setLiftState] = useState(false)
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  var base_path = [];
  var base_trail = [];
  var lastPose = undefined;

  useEffect(() => {
    if (manager === null) {
      return;
    }

    const resizeObserver = new ResizeObserver((entries) => {
      var img = entries[0].target; 
      var width = (img.clientWidth / mapSize.width) * 1.38;
      var height = (img.clientHeight / mapSize.height) * 1.9;

      updatePath(base_path, setPath, height, width);
      updateTrail(base_trail, setTrail, height, width);

      if (lastPose) {
        var ang = -lastPose[2]+Math.PI/8

        setVehiclePose([lastPose[1]*height,lastPose[0]*width, ang]);
      }
    });

    const displayRobot = (data) => {
      if (data.map) {
        const pose = data.map.substring(1, data.map.length - 1);
        const content = pose.split(",").map(function (item) {
          return parseFloat(item);
        });

        let resize_factor = 1, offset_x = 0, offset_y = 0;
        if (map == Map2) {
          resize_factor = 0.62;
          offset_x = 59;
          offset_y = 36;
        }

        let convPose = [content[0]*resize_factor + offset_x, content[1]*resize_factor + offset_y]

        lastPose = convPose

        var img = document.getElementById('exercise-img'); 
        //or however you get a handle to the IMG
        var width = (img.clientWidth / mapSize.width) * 1.38;
        var height = (img.clientHeight / mapSize.height) * 1.9;

        updateTrail(base_trail, setTrail, height, width);

        var ang = -content[2]

        setVehiclePose([convPose[1]*height,convPose[0]*width, ang]);
        addToTrail(convPose[1], convPose[0], base_trail);
      }

      if(data.image) {
        let canvas = document.getElementById("exercise-img");
          //Parse encoded image data and decode it
        function decode_utf8(s) {
            return decodeURIComponent(escape(s))
        }
        var image_data = JSON.parse(data.image),
        source = decode_utf8(image_data.image),
        shape = image_data.shape;

        if(source !== ""){
          setMap("data:image/png;base64," + source)
          // canvas.src = "data:image/png;base64," + source;
          // canvas.width = shape[1];
          // canvas.height = shape[0];
        }
      }
    };

    const displayPath = (data) => {
      if(data.array){
        var img = document.getElementById('exercise-img'); 
        //or however you get a handle to the IMG
        var width = (img.clientWidth / mapSize.width) * 1.38;
        var height = (img.clientHeight / mapSize.height) * 1.9;

        base_path = JSON.parse(data.array);
        updatePath(base_path, setPath, height, width)

        let target = base_path.at(-1)
        setTargetPose([(target[1]*height*100)/img.clientHeight, (target[0]*width*100)/img.clientWidth])
      }
    };

    const displayLiftSquare = (data) => {
      if(data.liftState === false || data.liftState){
        setLiftState(data.liftState)
        console.log(data.liftState)
      }
    };

    const updateCallback = (message) => {
      const data = message.data.update;
      displayRobot(data)
      displayPath(data)
      displayLiftSquare(data)

      // Send the ACK of the msg
      manager.send("gui", "ack");
    };

    const stateCallback = (message) => {
      if (message.data.state === "tools_ready") {
        let world = manager.getUniverse();
        
        if (world.includes("2")) {
          setMap(Map2)
          setMapSize(Map2Size)
        } else {
          setMap(Map1)
          setMapSize(Map1Size)
        }

        if (world.includes("Ackermann")) {
          setVehicleType(1)
        } else {
          setVehicleType(0)
        }

        try {
          base_path = []
          base_trail = []
          setPath("")
          setTrail("")
          setVehiclePose(null)
          setTargetPose(null)
        } catch (error) {
        }
      }
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
      <img src={map} alt="" className="exercise-canvas" id="exercise-img"/>
      {vehiclePose && vehicleType == 0 &&
        <div id="vehic-pos" className={(liftState) ? "lifting" : ""}
          style={{rotate: "z "+ (vehiclePose[2] +  Math.PI/2)+"rad", top: vehiclePose[0] -10 , left: vehiclePose[1] -10}}
        >
          <div className="arrow"/>
        </div>
      }
      {vehiclePose && vehicleType == 1 &&
        <div id="vehic-pos-ack"
           className={(liftState) ? "lifting-ack" : ""}
          style={{rotate: "z "+ (vehiclePose[2] + Math.PI)+"rad", top: vehiclePose[0] -25 , left: vehiclePose[1] -10}}
        >
          <div className="arrow-ack arrow"/>
        </div>
      }
      {trail &&
        <svg height="100%" width="100%" xmlns="http://www.w3.org/2000/svg" style={{zIndex:3, position:"absolute"}}>
          <path xmlns="http://www.w3.org/2000/svg" d={trail} 
            style={{strokeWidth: "2px", strokeLinejoin:"round", stroke: "blue", fill: "none"}}
          />
        </svg>
      }
      {path &&
        <svg height="100%" width="100%" xmlns="http://www.w3.org/2000/svg" style={{zIndex:2, position:"absolute"}}>
          <path xmlns="http://www.w3.org/2000/svg" d={path} 
            style={{strokeWidth: "2px", strokeLinejoin:"round", stroke: "green", fill: "none"}}
          />
        </svg>
      }
      {targetPose &&
        <div className="target" style={{top: `${targetPose[0]}%`, left: `calc(${targetPose[1]}% - ${10}px)`}}/>
      }
    </div>    
  );
}

export default WebGUI