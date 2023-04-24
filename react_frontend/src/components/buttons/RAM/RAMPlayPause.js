import LoadingButton from "@mui/lab/LoadingButton";
import React, { useContext, useEffect, useState } from "react";
import PropTypes from "prop-types";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import { Button } from "@mui/material";
import PauseIcon from "@mui/icons-material/Pause";

export const PlayPause = (props) => {
  const { editorCode, setLinterMessage } = useContext(props.context);
  const [disabledPlay, setDisabledPlay] = useState(true);
  const [disabledPause, setDisabledPause] = useState(true);

  const [buttonShow, setButtonShow] = useState("play");
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "running") {
        setButtonShow("pause");
        setDisabledPlay(true);
        setDisabledPause(false);
      } else if (
        message.data.state === "idle" ||
        message.data.state === "connected"
      ) {
        setButtonShow("play");
        setDisabledPlay(true);
        setDisabledPause(true);
      } else {
        setButtonShow("play");
        setDisabledPlay(false);
        setDisabledPause(true);
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
      .catch((response) => {
        console.error(response);
        setLoading(false);
      });
  };

  if (buttonShow === "play") {
    return (
      <LoadingButton
        disabled={disabledPlay}
        id={"loadIntoRobot"}
        loading={loading}
        color={"secondary"}
        onClick={() => {
          loadCode();
        }}
        sx={{ m: 0.5 }}
        variant={"outlined"}
      >
        {loading ? "_" : <PlayArrowIcon />}
      </LoadingButton>
    );
  } else {
    return (
      <Button
        disabled={disabledPause}
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
        sx={{ m: 0.5 }}
        variant={"outlined"}
      >
        <PauseIcon />
      </Button>
    );
  }
};
PlayPause.propTypes = {
  context: PropTypes.any,
};
