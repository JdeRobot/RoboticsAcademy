import * as React from "react";

import "./css/Reconstruction3DRR.css";
import { draw, reset_all } from "./helpers/helperRecontruction";

const SpecificRecontruction3D = (props) => {
    React.useEffect(() => {
        console.log("TestShowScreen subscribing to ['update'] events");
        const callback = (message) => {
          if(message.data.update.img1){
            const data = message.data.update;
            draw(data)
          }
          
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
          if (message.data.state === "ready") {
            try {
              reset_scene3d();
              reset_all()
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
    <div style={{display: "flex", height: "100%", width: "100%"}}>
       <div id="canvas" align = "center"  style={{flex: "1 1 auto", height: "100%", width: "100%"}}></div>
      <canvas id="gui_canvas"></canvas>
    </div>
  );
};

export default SpecificRecontruction3D;