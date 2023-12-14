import * as React from "react";
import PropTypes from "prop-types";
import {clearMap, draw, drawUserPosition, printParticles} from "./helpers/birds_eye"


function SpecificMontecarloLaserLoc(props) {
  const guiCanvasRef = React.useRef();

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");

    const callback = (message) => {
      const updateData = message.data.update;
      // Lógica para manejar el mapa
      if (updateData.map) {
        const pose = updateData.map.substring(1, updateData.map.length - 1);
        const content = pose.split(",").map(item => parseFloat(item));
        const poseUser = updateData.user.substring(1, updateData.user.length - 1);
        const userContent = poseUser.split(",").map(item => parseFloat(item));

        draw(
          guiCanvasRef.current,
          content[0],
          content[1],
          content[2],
          content[3],
          userContent[0],
          userContent[1],
          userContent[2],
          userContent[3]
        );
      }

      
      if (updateData.particles){
        const particles = JSON.parse(updateData.particles);
        if(particles != "") {
            printParticles(guiCanvasRef.current, particles);
        }
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
          clearMap(guiCanvasRef.current,)
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
    <div style={{ display: "flex", width: "100%", height: "100%" }}>
      <canvas
        ref={guiCanvasRef}
        style={{
          backgroundImage:
            "url('/static/exercises/montecarlo_visual_loc_newmanager/resources/mapgrannyannie.png')",
          border: "2px solid #d3d3d3",
          backgroundRepeat: "no-repeat",
          backgroundSize: "100% 100%",
          
        }}
      />
    </div>
  );
}


SpecificMontecarloLaserLoc.propTypes = {
  circuit: PropTypes.string,
};

export default SpecificMontecarloLaserLoc
