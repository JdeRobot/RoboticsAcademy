import React, { useEffect, useState } from "react";
// import PropTypes from "prop-types";
import LoadingButton from "@mui/lab/LoadingButton";
import ReplayIcon from "@mui/icons-material/Replay";

declare global {
  interface Window {
    RoboticsExerciseComponents: any;
  }
}

const ResetButton: React.FC = () => {
  const [disabled, setDisabled] = useState(true);
  const [loading, setLoading] = useState(false);
  useEffect(() => {
    const callback = (message: { data: { state: string } }) => {
      if (
        (message.data.state === "application_running") ||
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
      onClick={async () => {
        setLoading(true);
        try{
        await window.RoboticsExerciseComponents.commsManager
          .terminate_application();
        }catch(error){
        console.log(error);
        }finally{
        console.log("Reseted")
        setLoading(false);
        }
      }}
      sx={{ m: 0.5 }}
      variant={"outlined"}
    >
      <ReplayIcon />
    </LoadingButton>
  );
};


export default ResetButton;
