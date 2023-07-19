import * as React from "react";
import PropTypes from "prop-types";


function SpecificObstacleAvoidance(props) {
  const guiCanvasRef = React.useRef();

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");

    const callback = (message) => {
      const data = message.data.update;
      console.log(message.data.update);
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
    <img id="local-map"></img>
  );
}

SpecificObstacleAvoidance.propTypes = {
  circuit: PropTypes.string,
};

export default SpecificObstacleAvoidance
