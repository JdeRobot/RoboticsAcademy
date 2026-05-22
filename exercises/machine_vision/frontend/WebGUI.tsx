import React, { useState } from "react";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import WebGUIImage from "Components/exercise/WebGUIImage";

const WebGUI = () => {
  const exerciseContext = useExercise();
  const manager = exerciseContext.manager;

  const [image, setImage] = useState<string | undefined>(undefined);

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;

    if (data.update?.image_right) {
      const img = JSON.parse(data.update.image_right);

      if (img.image_right !== "") {
        setImage(`data:image/jpeg;base64,${img.image_right}`);
      }
    }
  };

  const stateCallback = () => {};

  connectApplication(manager, updateCallback, stateCallback);

  return (
    <WebGUIContainer>
      <WebGUIImage style={{ width: "100%" }} src={image} />
    </WebGUIContainer>
  );
};

export default WebGUI;