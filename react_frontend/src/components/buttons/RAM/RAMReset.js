import React, { useEffect, useState } from "react";
import PropTypes from "prop-types";
import RestartAltOutlinedIcon from "@mui/icons-material/RestartAltOutlined";
import LoadingButton from "@mui/lab/LoadingButton";

const RAMReset = () => {
  const [disabled, setDisabled] = useState(true);
  const [loading, setLoading] = useState(false);
  useEffect(() => {
    const callback = (message) => {
      if (
        (message.data.state === "ready") |
        (message.data.state === "running") |
        (message.data.state === "paused")
      ) {
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
    <LoadingButton
      disabled={disabled}
      id={"play"}
      loading={loading}
      color={"secondary"}
      loadingPosition="start"
      onClick={() => {
        setLoading(true);
        window.RoboticsExerciseComponents.commsManager
          .reset()
          .then(() => {
            setLoading(false);
          })
          .catch((response) => console.log(response));
      }}
      startIcon={<RestartAltOutlinedIcon />}
      sx={{ m: 0.5 }}
      variant={"outlined"}
    >
      Reset
    </LoadingButton>
  );
};
RAMReset.propTypes = {
  context: PropTypes.any,
};

export default RAMReset;
