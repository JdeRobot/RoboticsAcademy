import * as React from "react";
import PropTypes from "prop-types";
import { draw } from "Helpers/BirdEye";

function SpecificVacuumCleaner(props) {
  const guiCanvasRef = React.useRef();

  React.useEffect(() => {
    console.log("SpecificVacuumCleaner subscribing to ['update'] events");

    const callback = (message) => {
      console.log(message);
      const data = message.data.update;
      const pos_msg = data.pos_msg;
      const ang_msg = data.ang_msg;

      draw(
        guiCanvasRef.current,
        pos_msg[0],
        pos_msg[1],
        ang_msg[0],
        ang_msg[1]
      );

      // Send the ACK of the img
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

  return (
    <canvas
      ref={guiCanvasRef}
      style={{
        backgroundImage:
          "url('/static/exercises/vacuum_cleaner_newmanager/resources/images/mapgrannyannie.png')",
        border: "2px solid #d3d3d3",
        backgroundRepeat: "no-repeat",
        backgroundSize: "100% 100%",
        width: "100%",
        height: "100%",
      }}
    />
  );
}

SpecificVacuumCleaner.propTypes = {
  circuit: PropTypes.string,
};

export default SpecificVacuumCleaner;
