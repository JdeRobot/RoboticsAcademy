import { Box, Modal, Typography } from "@mui/material";
import React, { useEffect, useState } from "react";

export const LinterModal = () => {
  const [linter, setLinter] = useState(false);

  useEffect(() => {
    const callback = (message) => {
      if (!message.data.linter) {
        setLinter("Code loaded");
      } else {
        let linterMessage = JSON.stringify(message.data.linter);
        setLinter(linterMessage);
      }
    };
    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.LINTER],
      callback
    );
    return () => {
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.LINTER],
        callback
      );
    };
  }, []);

  const style = {
    position: "absolute",
    top: "50%",
    left: "50%",
    transform: "translate(-50%, -50%)",
    width: 400,
    bgcolor: "background.paper",
    border: "2px solid #000",
    boxShadow: 24,
    p: 4,
  };

  return (
    <Modal
      open={!!linter}
      onClose={() => {
        setLinter(false);
      }}
      aria-labelledby="modal-modal-title"
      aria-describedby="modal-modal-description"
    >
      <Box sx={style}>
        <Typography>{linter}</Typography>
      </Box>
    </Modal>
  );
};
