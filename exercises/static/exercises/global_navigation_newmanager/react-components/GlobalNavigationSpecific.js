import * as React from "react";
import PropTypes from "prop-types";
import { clearPath, draw, drawTargetPosition, generatePath } from "./helpers/birds_eye_global_navigation";
import Car from "../resources/images/car-top-view.svg";
import CityMap from "../resources/images/cityLargenBin.png";

import "./css/GUICanvas.css";

function SpecificGlobalNavigation(props) {
  const guiCanvasRef = React.useRef();
  const [showImage, setShowImage] = React.useState(false);
  const [carPose, setCarPose] = React.useState(null)
  const [destination, setDestination] = React.useState(null)
  let showMap = false
  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");

    const getMapDataAndDraw = (data) => {
      if (data.map && showMap) {
        const pose = data.map.substring(1, data.map.length - 1);
        const content = pose.split(',').map(function(item) {
          return parseFloat(item);
        })
        // draw((content[0]), (content[1]), content[2], content[3]);

        var img = document.getElementById('exercise-img'); 
        //or however you get a handle to the IMG
        var width = img.clientWidth / 400;
        var height = img.clientHeight / 400;

        setCarPose([content[1]*height,content[0]*width, Math.PI - content[2]]);
      }
    }

    const getImageAndDisplay = (data) => {
      if(data.image) {
        let canvas = document.getElementById("gui-canvas-numpy");
          //Parse encoded image data and decode it
        function decode_utf8(s) {
            return decodeURIComponent(escape(s))
        }
        var image_data = JSON.parse(data.image),
        source = decode_utf8(image_data.image),
        shape = image_data.shape;

        if(source !== ""){
          canvas.src = "data:image/png;base64," + source;
          canvas.width = shape[1];
          canvas.height = shape[0];
        }
      }
    }

    const getPathAndDisplay = (data) => {  
      if(data.array && showMap){
        // generatePath(JSON.parse(data.array))
      }
    }

    const callback = (message) => {
      const data = message.data.update;
      getMapDataAndDraw(data)
      getImageAndDisplay(data)
      getPathAndDisplay(data)

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
      console.log(message.data.state)
      if (message.data.state === "visualization_ready") {
        showMap = false
        setShowImage(false)
        // clearPath()
      } else {
        showMap = true
        setShowImage(true)
      }
    };
    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
      callback
    );
    return () => {
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
        callback
      );
    };
  }, []);

  function destinationPicker2(event) {
    var img = document.getElementById('exercise-img'); 
    let rect = img.getBoundingClientRect();

    //or however you get a handle to the IMG
    var width = img.clientWidth / 400;
    var height = img.clientHeight / 400;

    let cursorX = (event.clientX - rect.left);
    let cursorY = (event.clientY - rect.top);

    let cursorXMap = cursorX / width;
    let cursorYMap = cursorY / height;

    setDestination([cursorY, cursorX])

    return [cursorXMap, cursorYMap];
  }

  return (
    <div style={{display: "flex",   width: "100%", height: "100%", position:"relative"}}>
    {/* <canvas
      ref={guiCanvasRef}
      id="globalnav-eye"
      style={{
        backgroundImage:
          "url('/static/exercises/global_navigation_newmanager/resources/images/cityLargenBin.png')",
          border: "2px solid #d3d3d3",
          backgroundRepeat: "no-repeat",
          backgroundSize: "100% 100%",
      }}
      width= "400"
      height= "400"
      onClick={function pickLoc(event){
        var data = destinationPicker(event)
        let coords = {"data" : data};
        try {
          window.RoboticsExerciseComponents.commsManager.send("gui", `pick${data}`)
        } catch (error) {
        }  
      }}
    /> */}
    <img src={CityMap} alt="" className="exercise-canvas" id="exercise-img"
    onClick={ function pickLoc(event){
      var data = destinationPicker2(event)
      try {
        window.RoboticsExerciseComponents.commsManager.send("gui", `pick${data}`)
      } catch (error) {
      }  
    }}
    />
    {carPose &&
      <img src={Car} id="car" style={{rotate: "z "+ carPose[2]+"rad", top: carPose[0] -5 , left: carPose[1] -5}}/>
    }
    {destination &&
      <div className="target" style={{top: destination[0], left: destination[1] - 10}}/>
    }
    { showImage ? 
      (
      <img id="gui-canvas-numpy" width="400" height="400" style={{
          position: "absolute",
          left: "50%",
          width: "50%",
          height: "100%",
      }}></img>
      ) : (
      <div id="gui-canvas-numpy-empty" width="400" height="400" style={{
          position: "absolute",
          left: "50%",
          width: "50%",
          height: "100%",
          backgroundColor: "#000000"
      }}></div>
      )
    }
    </div>
  );
}

SpecificGlobalNavigation.propTypes = {
  circuit: PropTypes.string,
};

export default SpecificGlobalNavigation
