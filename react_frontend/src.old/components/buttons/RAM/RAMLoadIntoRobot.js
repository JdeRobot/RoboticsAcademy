import SmartToyOutlinedIcon from "@mui/icons-material/SmartToyOutlined";
import React, { useContext, useEffect, useState } from "react";
import PropTypes from "prop-types";
import LoadingButton from "@mui/lab/LoadingButton";

const RAMLoadIntoRobot = (props) => {
  const { submitCode } = useContext(props.context);
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
        [window.RoboticsExerciseComponents.commsManager.events.LINTER],
        callback
      );
    };
  }, []);
  return (
    <LoadingButton
      disabled={disabled}
      id={"loadIntoRobot"}
      loading={loading}
      color={"secondary"}
      onClick={() => {
        setLoading(true);
        submitCode().then(() => {
          setLoading(false);
        });
      }}
      startIcon={<SmartToyOutlinedIcon />}
      sx={{ m: 0.5 }}
      variant={"outlined"}
      loadingPosition="start"
    >
      Load in robot
    </LoadingButton>
  );
};
RAMLoadIntoRobot.propTypes = {
  context: PropTypes.any,
};

export default RAMLoadIntoRobot;
