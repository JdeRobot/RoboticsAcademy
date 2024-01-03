import React, { useEffect, useState } from "react";
import { Box, Typography, Tooltip } from "@mui/material";
import "../../styles/Indicator.css";

export default function WorldIndicator(props) {
  const exerciseConfig = JSON.parse(
    document.getElementById("exercise-config").textContent
  );
  const circuitName = exerciseConfig[0].name;
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "visualization_ready") {
        setConnected(true);
      }
    };
    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
      callback
    );

    return () => {
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
        callback
      );
    };
  }, []);

  return (
    <Tooltip title="World Launched">
      <Box className={connected ? "ready" : "waiting"}>
        <p className="title">World</p>
        <Typography sx={{ fontSize: "0.8rem" }} className="word">
          {circuitName}
        </Typography>
      </Box>
    </Tooltip>
  );
}
