import { paintEvent } from "./map_view";
import React from "react"
import "./css/Lasers.css";
const Lasers = (props) => {
    const guiCanvasRef = React.useRef();
    React.useEffect(() => {
        const callback = (message) => {
            if(message.data.update.map){
            const map_data = JSON.parse(message.data.update.map);
            console.log(JSON.parse(message.data.update.map))
            paintEvent(guiCanvasRef.current, map_data.car, map_data.obstacle, map_data.average, map_data.lasers, map_data.ranges)
            }
        };
        RoboticsExerciseComponents.commsManager.subscribe(
          [RoboticsExerciseComponents.commsManager.events.UPDATE],
          callback
        );
    
        return () => {
          console.log("TestShowScreen unsubscribing from ['state-changed'] events");
          RoboticsExerciseComponents.commsManager.unsubscribe(
            [RoboticsExerciseComponents.commsManager.events.UPDATE],
            callback
          );
        };
      }, []);
    return (
       <div      style={{
        width: "100%",
        height: "100%"
      }}>
		<canvas ref={guiCanvasRef} id="local-map-lasers" width="1280" height="720" ></canvas>
	</div>
    )
}

export default Lasers;