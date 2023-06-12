import React, { useEffect } from "react";
import { useState } from "react";
import Brightness1Icon from "@mui/icons-material/Brightness1";
import { Box, CircularProgress, Typography } from "@mui/material";

export const LaunchIndicator = () => {
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const callback = (message) => {
      if (
        (message.data.state === "ready") |
        (message.data.state === "paused") |
        (message.data.state === "running")
      ) {
        setConnected(true);
      } else {
        setConnected(false);
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
    <>
      {" "}
      <Typography>Ready</Typography>
      {connected ? (
        <Box sx={{ display: "flex", gap: "5px", alignItems: "center" }}>
          <Brightness1Icon
            color={"success"}
            fontSize={"small"}
          ></Brightness1Icon>
        </Box>
      ) : (
        <CircularProgress color="loading" size={20} />
      )}
    </>
  );
};
