import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import WebGUILidar from "Components/exercise/WebGUILidar";
import { useExercise } from "Contexts/ExerciseContext";
import { states } from "jderobot-commsmanager";
import { useEffect, useState } from "react";
import { draw, reset_all } from "./helpers/helperRecontruction";

const WebGUI = () => {
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);
  const [reset, setReset] = useState(false);
  const [pointsToPaint, setPointsToPaint] = useState(undefined);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;
    const update = data.update;

    if (update.img1) {
      draw(update);
      const point = JSON.parse(update.pts);
      if (point != "") {
        setPointsToPaint(point);
      }
    }

    if (update.reset) {
      setReset(true);
      reset_all();
    }
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setReset(true);
      reset_all();
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  return (
    <WebGUIContainer>
      <WebGUILidar
        toPaint={pointsToPaint}
        reset={reset}
        setReset={setReset}
        id="canvas"
        shape="cube"
        style={{ height: "100%", width: "33%" }}
      />
      <canvas id="gui_canvas" style={{ width: "67%" }}></canvas>
    </WebGUIContainer>
  );
};

export default WebGUI;
