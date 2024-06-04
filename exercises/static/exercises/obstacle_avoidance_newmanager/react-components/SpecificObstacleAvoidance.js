import * as React from "react";
import PropTypes from "prop-types";
import { paintEvent } from "./helpers/map";


function SpecificObstacleAvoidance(props) {
  const guiCanvasRef = React.useRef();
  const [image, setImage] = React.useState(
    "https://via.placeholder.com/800x600.png?text=No%20image%20received%20from%20exercise"
  );

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");

    const callback = (message) => {
      const data = message.data.update;
      if(data.map){
        const dataToDraw = JSON.parse(data.map)
        paintEvent(dataToDraw.target, dataToDraw.car, dataToDraw.obstacle, dataToDraw.average, dataToDraw.laser, dataToDraw.max_range)
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

  return (
    <div style={{display: "flex",   width: "100%",
    height: "100%", backgroundColor: "#363233"}}>
    <canvas
      ref={guiCanvasRef}
      id="local-map"

      style={{
        backgroundColor: "#363233",
      	marginTop: "5px",
        width: "50%",
        height: "100%",
        margin: "auto"
    }}
    />
    </div>
  );
}

SpecificObstacleAvoidance.propTypes = {
  circuit: PropTypes.string,
};

export default SpecificObstacleAvoidance
