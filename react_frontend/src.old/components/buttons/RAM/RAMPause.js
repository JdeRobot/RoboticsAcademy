import { Button } from "@mui/material";
import React, { useState } from "react";
import PropTypes from "prop-types";
import PauseIcon from "@mui/icons-material/Pause";

const RAMPause = () => {
  const [disabled, setDisabled] = useState(true);
  React.useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "running") {
        setDisabled(false);
      } else {
        setDisabled(true);
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
    <Button
      disabled={disabled}
      id={"play"}
      color={"secondary"}
      onClick={() => {
        window.RoboticsExerciseComponents.commsManager
          .pause()
          .then(() => {
            console.log("paused");
          })
          .catch((response) => console.log(response));
      }}
      startIcon={<PauseIcon />}
      sx={{ m: 0.5 }}
      variant={"outlined"}
    >
      Pause
    </Button>
  );
};
RAMPause.propTypes = {
  context: PropTypes.any,
};

export default RAMPause;
