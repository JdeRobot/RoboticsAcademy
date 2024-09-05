import React, { useEffect, useRef } from "react";
import { Box } from "@mui/material";
import "./css/GUICanvas.css";
import UpdateView from "./visualizers/UpdateView";

const SpecificFollowLine = (props) => {
  const canvasRef = useRef(null);
  const updateRef = useRef({});

  useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");

    const callback = (message) => {
      const updateData = message.data.update;

      if (updateData) {
        console.log('Received update data');

        // Update
        updateRef.current = {
          image: updateData.image,
          lapTime: updateData.lap,
        };

        // ACK
        window.RoboticsExerciseComponents.commsManager.send("gui", "ack");
      }
    };

    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    return () => {
      console.log("TestShowScreen unsubscribing from ['update'] events");
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
        callback
      );
    };
  }, []);

  return (
    <Box sx={{ height: "100%" }}>
      <UpdateView
        updateRef={updateRef}
        width={props.width}
        height={props.height}
      />
    </Box>
  );
};

SpecificFollowLine.defaultProps = {
  width: 800,
  height: 600,
};

export default SpecificFollowLine;
