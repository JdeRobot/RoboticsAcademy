import { useState, useEffect } from "react";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";

const parseImage = (raw: string): string | undefined => {
  if (!raw) {
    return undefined;
  }
  const image = JSON.parse(raw);
  if (image.image === "" || !(image.shape instanceof Array)) {
    return undefined;
  }
  return `data:image/jpeg;base64,${image.image}`;
};

const WebGUI = () => {
  const exerciseContext = useExercise();
  const [leftImage, setLeftImage] = useState<string | undefined>(undefined);
  const [rightImage, setRightImage] = useState<string | undefined>(undefined);
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const updateCallback = (updateData: unknown) => {
    const update = (updateData as any).update;
    // Frequency and RTF messages go to the status bar
    if (update.image_left === undefined && update.image_right === undefined) {
      return;
    }
    const left = parseImage(update.image_left);
    if (left) {
      setLeftImage(left);
    }
    const right = parseImage(update.image_right);
    if (right) {
      setRightImage(right);
    }
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setLeftImage(undefined);
      setRightImage(undefined);
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  return (
    <WebGUIContainer>
      <WebGUIImage id="left_img" style={{ left: "0" }} src={leftImage} />
      <WebGUIImage id="right_img" style={{ left: "50%" }} src={rightImage} />
    </WebGUIContainer>
  );
};

export default WebGUI;
