import { Button } from "@mui/material";
import React, { useEffect, useState } from "react";
import PropTypes from "prop-types";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";

const RAMPlay = () => {
  const [disabled, setDisabled] = useState(true);
  useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "ready" || message.data.state === "paused") {
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
        [window.RoboticsExerciseComponents.commsManager.events.LINTER],
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
          .run()
          .then(() => {
            console.log("running");
          })
          .catch((response) => console.log(response));
      }}
      startIcon={<PlayArrowIcon />}
      sx={{ m: 0.5 }}
      variant={"outlined"}
    >
      Play
    </Button>
  );
};
RAMPlay.propTypes = {
  context: PropTypes.any,
};

export default RAMPlay;

{
  /* <button
onClick={() => {
  window.RoboticsExerciseComponents.commsManager
    .pause()
    .then(() => {
      console.log("paused");
    })
    .catch((response) => console.log(response));
}}
>
pause
</button>
<button
onClick={() => {
  window.RoboticsExerciseComponents.commsManager
    .run()
    .then(() => {
      console.log("running");
    })
    .catch((response) => console.log(response));
}}
>
play
</button>
<button
onClick={() => {
  window.RoboticsExerciseComponents.commsManager
    .resume()
    .then(() => {
      console.log("running");
    })
    .catch((response) => console.log(response));
}}
>
resume
</button> */
}
