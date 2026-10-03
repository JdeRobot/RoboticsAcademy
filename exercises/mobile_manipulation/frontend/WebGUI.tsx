import { useState, useEffect } from "react";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import DeliveryMap, { DeliveryState } from "./DeliveryMap";

// Depth panel on the left and color panel on the right
// The myAGV delivery world shows its warehouse map instead
const WebGUI = () => {
  const exerciseContext = useExercise();
  const [rightImage, setRightImage] = useState<string | undefined>(undefined);
  const [leftImage, setLeftImage] = useState<string | undefined>(undefined);
  const [delivery, setDelivery] = useState<DeliveryState | undefined>(
    undefined,
  );
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  // The map shows as soon as the delivery world loads, before any code runs
  const emptyDelivery = (): DeliveryState | undefined => {
    const world = manager?.getWorld();
    if (world === undefined || !world.includes("Delivery")) {
      return undefined;
    }
    return {
      pose: null,
      path: [],
      red_target: 0,
      blue_target: 0,
      red_score: 0,
      blue_score: 0,
    };
  };

  useEffect(() => {
    setDelivery(emptyDelivery());
  }, [manager]);

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;
    const update = data.update;
    if (update.delivery) {
      setDelivery(JSON.parse(update.delivery));
      return;
    }
    let image;
    if (update.image_right) {
      image = JSON.parse(update.image_right);
      if (image.image_right != "" && image.shape_right instanceof Array) {
        setRightImage(`data:image/png;base64,${image.image_right}`);
      }
    }
    if (update.image_left) {
      image = JSON.parse(update.image_left);
      if (image.image_left != "" && image.shape_left instanceof Array) {
        setLeftImage(`data:image/png;base64,${image.image_left}`);
      }
    }
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setLeftImage(undefined);
      setRightImage(undefined);
      setDelivery(emptyDelivery());
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  return (
    <WebGUIContainer>
      {delivery ? (
        <DeliveryMap state={delivery} />
      ) : (
        <>
          <WebGUIImage id="left_img" style={{ left: "0" }} src={leftImage} />
          <WebGUIImage
            id="right_img"
            style={{ left: "50%" }}
            src={rightImage}
          />
        </>
      )}
    </WebGUIContainer>
  );
};

export default WebGUI;
