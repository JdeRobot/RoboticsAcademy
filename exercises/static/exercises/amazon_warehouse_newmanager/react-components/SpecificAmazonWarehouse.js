import * as React from "react";
import PropTypes from "prop-types";
import { clearPath, clearMap, draw, drawTargetPosition, generatePath } from "./helpers/bird_eye_amazon_warehouse";

function SpecificAmazonWarehouse(props) {
  const guiCanvasRef = React.useRef();

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");

    const displayRobot = (data) => {
      if (data.map) {
        const pose = data.map.substring(1, data.map.length - 1);
        const content = pose.split(",").map(function (item) {
          return parseFloat(item);
        });
        let resize_factor = 1;
        let offset_x = 0;
        let offset_y = 0;
        if (guiCanvasRef.current.style.backgroundImage == "url('/static/exercises/amazon_warehouse_newmanager/resources/images/map_2.png')") {
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
    };

    const displayPath = (data) => {
      if(data.array){
        generatePath(JSON.parse(data.array), guiCanvasRef.current)
      }
    };

    const callback = (message) => {
      const data = message.data.update;
      displayRobot(data)
      displayPath(data)
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
        let world = document.getElementById("circuit-selector").innerText;
        if (world === "amazon_warehouse_ros2_world2_ackermann" || world === "amazon_warehouse_ros2_world2") {
          console.log(world)
          guiCanvasRef.current.style.backgroundImage = "url('/static/exercises/amazon_warehouse_newmanager/resources/images/map_2.png')";
        } else {
          guiCanvasRef.current.style.backgroundImage = "url('/static/exercises/amazon_warehouse_newmanager/resources/images/map.png')";
        }
        try {
          clearMap()
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
    <canvas
      ref={guiCanvasRef}
      id="amazon_map_canvas"
      style={{
        backgroundImage:
          "url('/static/exercises/amazon_warehouse_newmanager/resources/images/map.png')",
        border: "2px solid #d3d3d3",
        backgroundRepeat: "no-repeat",
        backgroundSize: "100% 100%",
        width: "100%",
        height: "100%"
      }}
    />
  );
}

SpecificAmazonWarehouse.propTypes = {
  circuit: PropTypes.string,
};

export default SpecificAmazonWarehouse
