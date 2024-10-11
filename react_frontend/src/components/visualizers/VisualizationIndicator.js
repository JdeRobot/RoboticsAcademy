import React, { useEffect, useState } from "react";
import { Box, Typography, Tooltip } from "@mui/material";
import "../../styles/Indicator.css";

export default function VisualizationIndicator(props) {
  const [visualizationReady, setVisualizationReady] = useState(false);

  useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "visualization_ready") {
        setVisualizationReady(true);
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
    <Tooltip title="Robotics Backend">
      <Box className={visualizationReady ? "ready" : "waiting"}>
        <p className="title">GUI</p>
        <Typography sx={{ fontSize: "0.8rem" }} className="word">
          {visualizationReady ? "ready" : "waiting"}
        </Typography>
      </Box>
    </Tooltip>
  );
}
