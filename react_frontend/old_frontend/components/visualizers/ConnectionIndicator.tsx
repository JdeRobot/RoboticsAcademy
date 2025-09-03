import React, { useEffect, useState } from "react";
import { Box, Typography, Tooltip } from "@mui/material";
import "Styles/Indicator.css";
import PropTypes from "prop-types";

const ConnectionIndicator: React.FC = () => {
  const [radiVersion, setRadiVersion] = useState<string>("");
  const [connected, setConnected] = useState<boolean>(false);

  useEffect(() => {
    const callback = (message: MessageEvent<any>) => {
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

  useEffect(() => {
    const callback = (message: MessageEvent<any>) => {
      setRadiVersion(message.data.robotics_backend_version);
    };
    window.RoboticsExerciseComponents.commsManager.suscribreOnce(
      [window.RoboticsExerciseComponents.commsManager.events.INTROSPECTION],
      callback
    );
  }, []);

  return (
    <Tooltip title="Robotics Backend">
      <Box className={connected ? "ready" : "waiting"}>
        <p className="title">Robotics Backend</p>
        <Typography sx={{ fontSize: "0.8rem" }} className="word">
          {radiVersion}
        </Typography>
      </Box>
    </Tooltip>
  );
}



export default ConnectionIndicator;
