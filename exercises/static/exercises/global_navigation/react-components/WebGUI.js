import { useState, useEffect} from "react";
import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import {updatePath} from "./helpers/GlobalNavigationHelper";
import Car from "../resources/images/car-top-view.svg";
import CityMap from "../resources/images/cityLargenBin.png";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer from "Components/exercise/WebGUIContainer";
import "./css/GUICanvas.css";

var coords = undefined;

function WebGUI(props) {
  const [carPose, setCarPose] = useState(null)
  const [destination, setDestination] = useState(null)
  const [path, setPath] = useState("")
  const [userImage, setUserImage] = useState(undefined);
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);
  
  var trail = [];
  var lastPose = undefined;
  let showMap = false;

  useEffect(() => {
    if (manager === null) {
      return;
    }

    const resizeObserver = new ResizeObserver((entries) => {
      var img = entries[0].target; 
      //or however you get a handle to the IMG
      var width = img.clientWidth / 400;
      var height = img.clientHeight / 400;

      updatePath(trail, setPath, height, width);

      if (lastPose) {
        setCarPose([lastPose[1]*height,lastPose[0]*width, Math.PI - lastPose[2]]);
      }
    });

    const getMapDataAndDraw = (data) => {
      if (data.map && showMap) {
        const pose = data.map.substring(1, data.map.length - 1);
        const content = pose.split(',').map(function(item) {
          return parseFloat(item);
        })
        lastPose = content;

        var img = document.getElementById('exercise-img'); 
        //or however you get a handle to the IMG
        var width = img.clientWidth / 400;
        var height = img.clientHeight / 400;

        setCarPose([content[1]*height,content[0]*width, Math.PI - content[2]]);
      }
    }

    const getImageAndDisplay = (data) => {
      if (data.image) {
        let image = JSON.parse(data.image);
        if (image.image != "" && image.shape instanceof Array) {
          setUserImage(`data:image/png;base64,${image.image}`);
        }
      }
    }

    const getPathAndDisplay = (data) => {  
      if(data.array && showMap){
        var img = document.getElementById('exercise-img'); 
        //or however you get a handle to the IMG
        var width = img.clientWidth / 400;
        var height = img.clientHeight / 400;

        trail = JSON.parse(data.array);
        updatePath(trail, setPath, height, width);
      }
    }

    const updateCallback = (message) => {
      const data = message.data.update;
      console.log(data)
      getMapDataAndDraw(data)
      getImageAndDisplay(data)
      getPathAndDisplay(data)

      // Send the ACK of the msg
      manager.send("gui", "ack");
    };

    const stateCallback = (message) => {
      console.log(message.data.state)
      if (message.data.state === "tools_ready") {
        showMap = false
        trail = []
        setPath("")
        setUserImage()
      } else {
        showMap = true
        // Resend Target
        console.log("Resend target:", coords)
        if (coords) {
          try {
            manager.send("gui", `pick${coords}`)
          } catch (error) {
          }  
        }
      }
    };

    resizeObserver.observe(document.getElementById("exercise-img"));

    manager.subscribe(events.UPDATE, updateCallback);
    manager.subscribe(events.STATE_CHANGED, stateCallback);

    return () => {
      manager.unsubscribe(events.UPDATE, updateCallback);
      manager.unsubscribe(events.STATE_CHANGED, stateCallback);
    };
  }, [manager]);

  function destinationPicker(event) {
    var img = document.getElementById('exercise-img'); 
    let rect = img.getBoundingClientRect();

    //or however you get a handle to the IMG
    var width = img.clientWidth / 400;
    var height = img.clientHeight / 400;

    let cursorX = (event.clientX - rect.left);
    let cursorY = (event.clientY - rect.top);
    
    let cursorXMap = cursorX / width;
    let cursorYMap = cursorY / height;

    setDestination([(cursorY*100)/img.clientHeight, (cursorX*100)/(img.clientWidth*2)])
    return [cursorXMap, cursorYMap];
  }

  return (
    <WebGUIContainer>
      <img src={CityMap} alt="" className="exercise-canvas" id="exercise-img"
        onClick={ function pickLoc(event){
          var data = destinationPicker(event)
          coords = data
          try {
            manager.send("gui", `pick${data}`)
          } catch (error) {
          }  
        }}
      />
      {carPose &&
        <img src={Car} id="car" style={{rotate: "z "+ carPose[2]+"rad", top: carPose[0] -5 , left: carPose[1] -5}}/>
      }
      {destination &&
        <div className="target" style={{top: `${destination[0]}%`, left: `calc(${destination[1]}% - ${10}px)`}}/>
      }
      {path &&
        <svg height="100%" width="50%" xmlns="http://www.w3.org/2000/svg" style={{zIndex:2}}>
          <path xmlns="http://www.w3.org/2000/svg" d={path} 
            style={{strokeWidth: "2px", strokeLinejoin:"round", stroke: "green", fill: "none"}}
          />
        </svg>
      }
      <WebGUIImage id="user-img" style={{ left: "50%" }} src={userImage} />
    </WebGUIContainer>
  );
}

export default WebGUI
