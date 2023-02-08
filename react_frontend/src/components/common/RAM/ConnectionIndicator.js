import LoadingButton from "@mui/lab/LoadingButton";
import ConnectingAirportsIcon from "@mui/icons-material/ConnectingAirports";
import React, { useEffect } from "react";
import { useState } from "react";

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
    <LoadingButton
      id={"connection-button"}
      startIcon={<ConnectingAirportsIcon />}
      variant="contained"
      loadingPosition="start"
      color={!connected ? "loading" : "success"}
      sx={{ marginX: 1 }}
      size={"small"}
    >
      Connected
    </LoadingButton>
  );
};
