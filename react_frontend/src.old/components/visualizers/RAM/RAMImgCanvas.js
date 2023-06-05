import * as React from "react";
import PropTypes from "prop-types";

export default function ImgCanvas() {
  const [image, setImage] = React.useState(
    "https://via.placeholder.com/800x600.png?text=No%20image%20received%20from%20exercise"
  );

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");

    const callback = (message) => {
      const update = message.data.update;
      if (update.image) {
        const image = JSON.parse(update.image);
        setImage(`data:image/png;base64,${image.image}`);
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
    <>
      <img height={"400px"} width={"100%"} src={image} id="gui_canvas" />
    </>
  );
}

ImgCanvas.propTypes = {
  context: PropTypes.any,
};
