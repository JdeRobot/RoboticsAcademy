import { Box, Typography } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";

export default function ImgCanvas(props) {
  const [image, setImage] = React.useState(
    "https://via.placeholder.com/800x600.png?text=No%20image%20received%20from%20exercise"
  );

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");
    setImage(
      "https://via.placeholder.com/800x600.png?text=No%20image%20received%20from%20exercise"
    );

    const callback = (message) => {
      const update = message.data.update;
      if (update.image) {
        const image = JSON.parse(update.image);
        setImage(`data:image/png;base64,${image.image}`);
      } else {
        setImage(
          "https://via.placeholder.com/800x600.png?text=No%20image%20received%20from%20exercise"
        );
      }
    };

    RoboticsExerciseComponents.commsManager.subscribe(
      [RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    return () => {
      console.log("TestShowScreen unsubscribing from ['state-changed'] events");
      RoboticsExerciseComponents.commsManager.unsubscribe(
        [RoboticsExerciseComponents.commsManager.events.UPDATE],
        callback
      );
    };
  }, []);
  return (
    <Box
      sx={{
        m: 3,
        p: 2,
        display: "flex",
        flexDirection: "column",
        border: "2px solid",
        alignItems: "center",
      }}
    >
      <Typography>{props.heading}</Typography>
      <img
        height={250}
        width={500}
        sx={{
          // marginX: 15,
          // marginY: 5,
          border: "2px solid #d3d3d3",
          backgroundRepeat: "no-repeat",
        }}
        src={image}
        id="gui_canvas"
      />
    </Box>
  );
}

ImgCanvas.propTypes = {
  context: PropTypes.any,
  heading: PropTypes.string,
};
