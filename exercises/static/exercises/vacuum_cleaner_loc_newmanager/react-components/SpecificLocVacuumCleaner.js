import * as React from "react";
import PropTypes from "prop-types";
import {updatePath, addToPath} from "./helpers/VacuumCleanerHelper";
import houseMapClean from "../resources/images/mapgrannyannie_clean.png";
import houseMapDirty from "../resources/images/mapgrannyannie_dirty.png";
import Vacuum from "../resources/images/vacuum.svg";

import "./css/GUICanvas.css";

export default function SpecificLocVacuumCleaner() {
  const [vacuumPose, setVacuumPose] = React.useState(null)
  const [path, setPath] = React.useState("")
  var trail = [];
  var lastPose = undefined;

  const resizeObserver = new ResizeObserver((entries) => {
    var img = entries[0].target; 
    //or however you get a handle to the IMG
    var width = (1012 / 300) / (1012 /img.clientWidth);
    var height = (1012 / 150) / (1012 /img.clientHeight);

    updatePath(trail, setPath, height, width);

    if (lastPose) {
      setVacuumPose([lastPose[1]*height,lastPose[0]*width, -lastPose[2]]);
    }
  });

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");

    const callback = (message) => {
      const updateData = message.data.update;
      // Lógica para manejar el mapa
      if (updateData.map) {
        const pose = updateData.map.substring(1, updateData.map.length - 1);
        const content = pose.split(",").map(item => parseFloat(item));
        lastPose = content;

        var img = document.getElementById('exercise-img'); 
        //or however you get a handle to the IMG
        var width = (1013 / 300) / (1013 /img.clientWidth);
        var height = (1012 / 150) / (1012 /img.clientHeight);

        updatePath(trail, setPath, height, width);

        setVacuumPose([content[1]*height,content[0]*width, -content[2]]);
        addToPath(content[1], content[0], trail);
      }

      if(updateData.image) {
        let canvas = document.getElementById("gui-canvas-numpy");
          //Parse encoded image data and decode it
        function decode_utf8(s) {
            return decodeURIComponent(escape(s))
        }
        var image_data = JSON.parse(updateData.image),
        source = decode_utf8(image_data.image),
        shape = image_data.shape;

        if(source !== ""){
          canvas.src = "data:image/png;base64," + source;
          canvas.width = shape[1];
          canvas.height = shape[0];
        }
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
      console.log(message);
      if (message.data.state === "visualization_ready") {
        try {
          setPath("")
          trail = []
        } catch (error) {
        }
      }
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
      <img src={houseMapDirty} alt="" className="exercise-canvas" id="exercise-img"/>
      <div className="overlay" id="map-container">
        {vacuumPose &&
          <div id="vacuum-pos" style={{rotate: "z "+ vacuumPose[2]+"rad", top: vacuumPose[0] -10 , left: vacuumPose[1] -10}}>
            <img src={Vacuum} id="vacuum-pos"/>
            <div className="arrow"/>
          </div>
        }
        <svg height="100%" width="100%" xmlns="http://www.w3.org/2000/svg">
          <mask id="svg-draw" xmlns="http://www.w3.org/2000/svg">
            {path ? (
              <path xmlns="http://www.w3.org/2000/svg" d={path} style={{strokeWidth: "20px", strokeLinejoin:"round", stroke: "white", fill: "none"}}/>
            ) : (
              <path xmlns="http://www.w3.org/2000/svg"></path>
            )}
          </mask>
          <image href={houseMapClean} height="100%" width="100%" mask="url(#svg-draw)" preserveAspectRatio="none"></image>
        </svg>
      </div>
      <img id="gui-canvas-numpy" width="400" height="400" style={{
            position: "absolute",
            left: "50%",
            width: "50%",
            height: "100%",
      }}></img>
    </div>
  );
}

SpecificLocVacuumCleaner.propTypes = {
  circuit: PropTypes.string,
};
