import React, { useEffect, useState } from "react";
import { Box, Typography, Tooltip } from "@mui/material";
import "../../styles/Indicator.css";
import PropTypes from "prop-types";

function ConnectionIndicator() {
  const [radiVersion, setRadiVersion] = useState("");
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

  useEffect(() => {
    const callback = (message) => {
      setRadiVersion(message.data.radi_version);
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

ConnectionIndicator.propTypes = {
  exerciseName: PropTypes.string,
};

export default ConnectionIndicator;
