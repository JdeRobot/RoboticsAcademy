import { useState, useEffect } from "react";
import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import "./css/Reconstruction3DRR.css";
import { draw, reset_all } from "./helpers/helperRecontruction";

const Reconstruction3D = () => {
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  useEffect(() => {
    if (manager === null) {
      return;
    }

    const updateCallback = (message) => {
      if (message.data.update.img1) {
        const data = message.data.update;
        draw(data);
      }
      // Send the ACK of the msg
      manager.send("gui", "ack");
    };

    const stateCallback = (message) => {
      if (message.data.state === "ready") {
        try {
          reset_scene3d();
          reset_all();
        } catch (error) {}
      }
    };

    manager.subscribe(events.UPDATE, updateCallback);
    manager.subscribe(events.STATE_CHANGED, stateCallback);

    return () => {
      manager.unsubscribe(events.UPDATE, updateCallback);
      manager.unsubscribe(events.STATE_CHANGED, stateCallback);
    };
  }, [manager]);

  return (
    <div
      style={{
        display: "flex",
        height: "100%",
        width: "100%",
        position: "relative",
      }}
    >
      <div id="canvas" style={{ height: "100%", width: "33%" }} />
      <canvas id="gui_canvas" style={{ width: "67%" }}></canvas>
    </div>
  );
};

export default Reconstruction3D;
