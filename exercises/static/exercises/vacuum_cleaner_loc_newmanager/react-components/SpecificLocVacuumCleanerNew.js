import * as React from "react";
import PropTypes from "prop-types";
import { draw, clearMap } from "Helpers/BirdEye";
import houseMap from "../resources/images/mapgrannyannie.png";

import "./css/GUICanvas.css";

function name(params) {
  
}

export default function SpecificLocVacuumCleaner() {
  const guiCanvasRef = React.useRef();
  const [vacuumPose, setVacuumPose] = React.useState(null)

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");

    const callback = (message) => {
      const updateData = message.data.update;
      // Lógica para manejar el mapa
      if (updateData.map) {
        const pose = updateData.map.substring(1, updateData.map.length - 1);
        const content = pose.split(",").map(item => parseFloat(item));

        // draw(
        //   guiCanvasRef.current,
        //   content[0],
        //   content[1],
        //   content[2],
        //   content[3],
        // );
        var img = document.getElementById('exercise-img'); 
        //or however you get a handle to the IMG
        var width = (1013 / 300) / (1013 /img.clientWidth);
        var height = (1012 / 150) / (1012 /img.clientHeight);

        console.log(content[2])

        setVacuumPose([content[1]*height,content[0]*width, -content[2] + 2*Math.PI]);
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
          clearMap(guiCanvasRef.current,)
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
      {/* <canvas
        ref={guiCanvasRef}
        style={{
          backgroundImage:
            "url('/static/exercises/vacuum_cleaner_loc_newmanager/resources/images/mapgrannyannie.png')",
          backgroundRepeat: "no-repeat",
          backgroundSize: "100% 100%",
          border: "2px solid #d3d3d3",
          width: "50%",
          height: "100%",
        }}
      /> */}
      <img src={houseMap} alt="" className="exercise-canvas" id="exercise-img"/>
      <div className="overlay" id="map-container">
        {vacuumPose &&
          <div id="vacuum-pos" style={{rotate: "z "+ vacuumPose[2]+"rad", top: vacuumPose[0] -10 , left: vacuumPose[1] -10}}>
            <div className="arrow"/>
          </div>
        }
      </div>
      <img id="gui-canvas-numpy" width="400" height="400" style={{
            marginTop: "5px",
            width: "100%",
            height: "100%",
            margin: "auto"
      }}></img>
    </div>
  );
}

SpecificLocVacuumCleaner.propTypes = {
  circuit: PropTypes.string,
};
