import React, { useContext, useEffect, useState } from "react";
import PropTypes from "prop-types";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import LoadingButton from "@mui/lab/LoadingButton";

const RAMPlay = (props) => {
  const { editorCode, setLinterMessage } = useContext(props.context);
  const [disabled, setDisabled] = useState(true);
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    const callback = (message) => {
      if (
        message.data.state === "ready" ||
        message.data.state === "paused" ||
        message.data.state === "running"
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

  const loadCode = () => {
    setLoading(true);
    window.RoboticsExerciseComponents.commsManager
      .send("load", {
        code: editorCode,
      })
      .then(() => {
        runCode();
        setLoading(false);
      })
      .catch((response) => {
        let linterMessage = JSON.stringify(response.data.message).split("\\n");
        setLinterMessage(linterMessage);
        setLoading(false);
      });
  };

  const runCode = () => {
    window.RoboticsExerciseComponents.commsManager
      .run()
      .then(() => {
        console.log("running");
      })
      .catch((response) => console.error(response));
  };

  return (
    <LoadingButton
      disabled={disabled}
      id={"loadIntoRobot"}
      loading={loading}
      color={"secondary"}
      onClick={() => {
        loadCode();
      }}
      startIcon={<PlayArrowIcon />}
      sx={{ m: 0.5 }}
      variant={"outlined"}
      loadingPosition="start"
    >
      play
    </LoadingButton>
  );
};
RAMPlay.propTypes = {
  context: PropTypes.any,
};

export default RAMPlay;
