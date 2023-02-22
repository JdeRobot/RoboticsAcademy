import React, { useEffect } from "react";
import { useState } from "react";
import { Box, CircularProgress, Typography } from "@mui/material";
import Brightness1Icon from "@mui/icons-material/Brightness1";

export const ConnectionIndicator = () => {
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "connected") {
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
    <>
      <Typography>Connected</Typography>
      {connected ? (
        <Box
          sx={{
            display: "flex",
            gap: "5px",
            alignItems: "center",
          }}
        >
          <Brightness1Icon
            color={!connected ? "loading" : "success"}
            fontSize={"small"}
          ></Brightness1Icon>
        </Box>
      ) : (
        <CircularProgress color="loading" size={20} />
      )}
    </>
  );
};
