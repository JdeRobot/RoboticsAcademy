import * as React from "react";
import PropTypes from "prop-types";
import { draw } from "Helpers/BirdEye";

function SpecificGlobalNavigation(props) {
  const guiCanvasRef = React.useRef();

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");

    const callback = (message) => {
      const data = message.data.update;
      if (data.map) {
        const pose = data.map.substring(1, data.map.length - 1);
        const content = pose.split(",").map(function (item) {
          return parseFloat(item);
        });
        draw(
          guiCanvasRef.current,
          content[0],
          content[1],
          content[2],
          content[3]
        );
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
    <canvas
      ref={guiCanvasRef}
      style={{
        backgroundImage:
          "url('/static/exercises/global_navigation_newmanager/resources/images/cityLargenBin.png')",
        border: "2px solid #d3d3d3",
        backgroundRepeat: "no-repeat",
        backgroundSize: "100% 100%",
        width: "100%",
        height: "100%"
      }}
    />
  );
}

SpecificGlobalNavigation.propTypes = {
  circuit: PropTypes.string,
};

export default SpecificGlobalNavigation
