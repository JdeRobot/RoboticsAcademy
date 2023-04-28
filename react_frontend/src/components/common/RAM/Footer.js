import { Typography } from "@mui/material";
import { Box } from "@mui/system";
import React, { useEffect, useState } from "react";
export const Footer = () => {
  const [ramVersion, setramVersion] = useState(null);
  useEffect(() => {
    const callback = (message) => {
      if (message) {
        setramVersion(message.data.version);
      }
    };

    window.RoboticsExerciseComponents.commsManager.suscribreOnce(
      [window.RoboticsExerciseComponents.commsManager.events.VERSION],
      callback
    );
  });
  return (
    <Box
      sx={{
        display: "flex",
        justifyContent: "center",
        alignItems: "start",
        height: "50px",
      }}
    >
      {ramVersion ? <Typography>Running RADI: {ramVersion}</Typography> : null}
    </Box>
  );
};
