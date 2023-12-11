import * as React from "react";
import PropTypes from "prop-types";
import {  clearMap, draw, generatePath } from "./helpers/bird_eye_amazon_warehouse";

export default function SpecificAmazonWarehouse() {
  const guiCanvasRef = React.useRef();

  
    React.useEffect(() => {
      console.log("TestShowScreen subscribing to ['update'] events");

      const callback = (message) => {
          const data = message.data.update;
          console.log(data)


          // Parse the Map data
          // Slice off ( and )
          if (data.map) {
              const pose = data.map.substring(1, data.map.length - 1);
              const content = pose.split(",").map(function (item) {
                  return parseFloat(item);
              });
              let resize_factor = 1;
              let offset_x = 0;
              let offset_y = 0;
              if (guiCanvasRef.current.style.backgroundImage == 'url("/static/assets/img/amazon_warehouse_map_2.png")') {
                resize_factor = 0.62;
                offset_x = 59;
                offset_y = 36;
              }              
              draw(
                  guiCanvasRef.current,
                  content[0] * resize_factor + offset_x,
                  content[1] * resize_factor + offset_y,
                  content[2] * resize_factor,
                  content[3] * resize_factor
              );
          }
          if(data.array){
            generatePath(JSON.parse(data.array), guiCanvasRef.current)
          }
      };
 

      window.RoboticsExerciseComponents.commsManager.subscribe(
          [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
          callback
      );

      return () => {
          console.log("TestShowScreen unsubscribing from ['update'] events");
          window.RoboticsExerciseComponents.commsManager.unsubscribe(
              [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
              callback
          );
      };
  }, []);


  React.useEffect(() => {
      console.log("TestShowScreen subscribing to ['state-changed'] events");

      const callback = (message) => {
          if(message.data.state != "running") {
              clearMap(guiCanvasRef.current)
          }
      };

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
  }, []);


  return (
    <div style={{ 
        position: 'relative', 
        width: '100%', 
        height: '100%', 
        display: 'inline-block'}}>
    <p   style={{
      position: 'absolute',
      top: 0,
      right: 20,
      margin: '10px',
      fontSize: '18px',
      color: 'white',
      zIndex: 1,}}>
        Score:</p>
    <p id="score" 
    style={{
      position: 'absolute',
      top: 0,
      right: 0,
      margin: '10px',
      fontSize: '18px',
      color: 'white',
      zIndex: 1,
      
    }}>0</p>
    <canvas
      ref={guiCanvasRef}
      id="amazon_map_canvas"
      style={{        
        backgroundImage:
          "url('/static/exercises/amazon_warehouse_newmanager/resources/images/map.png')",        border: "2px solid #d3d3d3",
        backgroundRepeat: "no-repeat",
        backgroundSize: "100% 100%",
        width: "100%",
        height: "100%",
        
      }}
    />
    </div>
  );
}

SpecificAmazonWarehouse.propTypes = {
  circuit: PropTypes.string,
};


