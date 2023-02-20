import { Typography } from "@mui/material";
import { Box } from "@mui/system";
import React, { useState } from "react";

export const Frequencies = () => {
  const [frequencies, setFrequencies] = useState({ brain: 0, gui: 0, rtf: 0 });
  React.useEffect(() => {
    const callback = (message) => {
      const update = message.data.update;
      if (update.brain) {
        setFrequencies(update);
      }
    };

    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    return () => {
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
        callback
      );
    };
  }, []);

  return (
    <Box
      sx={{
        display: "flex",
        gap: "5px",

        paddingLeft: "25px",
        color: "blue",
      }}
    >
      <Typography title="BRAIN">{frequencies.brain}</Typography>
      <Typography title="GUI">{frequencies.gui}</Typography>
      <Typography title="RTF">{frequencies.rtf}</Typography>
    </Box>
  );
};
