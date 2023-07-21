import * as React from "react";

import "./css/Reconstruction3DRR.css";
import { draw } from "./helpers/ws_gui";

const SpecificRecontruction3D = (props) => {
    React.useEffect(() => {
        console.log("TestShowScreen subscribing to ['update'] events");
        const callback = (message) => {
          const data = message.data.update;
          console.log(data)
          draw(data)
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
  return (
    <div><canvas id="gui_canvas"></canvas></div>
  );
};

export default SpecificRecontruction3D;