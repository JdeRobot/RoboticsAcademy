import { useState, useEffect } from "react";
import { events, states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUI3D from "Components/exercise/WebGUI3D";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import "./css/Reconstruction3DRR.css";
import { draw, reset_all } from "./helpers/helperRecontruction";

const WebGUI = () => {
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);
  const [reset, setReset] = useState(false);
  const [pointsToPaint, setPointsToPaint] = useState(undefined);
  let connection = connectApplication(manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  useEffect(() => {
    if (manager === null) {
      return;
    }

    

    const updateCallback = (message) => {
      connection.end();
      if (message.data.update.img1) {
        const data = message.data.update;
        draw(data);
        var point = JSON.parse(data.pts);
        if(point != "")
        {
            setPointsToPaint(point);
        }
      }
      // Send the ACK of the msg
      manager.send("gui", "ack");
    };

    const stateCallback = (message) => {
      if (message.data.state === states.TOOLS_READY) {
        try {
          setReset(true)
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
    <WebGUIContainer>
      <WebGUI3D
        toPaint={pointsToPaint}
        reset={reset}
        setReset={setReset}
        id="canvas"
        style={{ height: "100%", width: "33%" }}
      />
      <canvas id="gui_canvas" style={{ width: "67%" }}></canvas>
    </WebGUIContainer>
  );
};

export default WebGUI;
