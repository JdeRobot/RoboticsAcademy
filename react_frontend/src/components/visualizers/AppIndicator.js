import React, { useEffect, useState } from "react";
import { Box, Typography, Tooltip } from "@mui/material";
import "../../styles/Indicator.css";

function AppIndicator(props) {
  const [running, setRunning] = useState(false);

  useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "running") {
        setRunning(true);
      } else {
        setRunning(false);
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
    <Tooltip title="Application Running">
      <Box className={running ? "ready" : "waiting"}>
        <p className="title">Application</p>
        <Typography sx={{ fontSize: "0.8rem" }} className="word">
          {props.name}
        </Typography>
      </Box>
    </Tooltip>
  );
}

export default AppIndicator;
