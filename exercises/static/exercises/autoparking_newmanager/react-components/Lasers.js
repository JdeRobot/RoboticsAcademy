import { paintEvent } from "./map_view";
import React from "react"

const Lasers = (props) => {
    React.useEffect(() => {
        const callback = (message) => {
            if(message.data.update.map){
            const map_data = JSON.parse(message.data.update.map);
            console.log(JSON.parse(message.data.update.map))
            paintEvent(map_data.car, map_data.obstacle, map_data.average, map_data.lasers, map_data.ranges)
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
        <canvas id="local-map-lasers"></canvas>
    )
}

export default Lasers;