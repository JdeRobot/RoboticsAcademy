import * as React from "react";
import PropTypes from "prop-types";
import { Box } from "@mui/system";
import { CircularProgress, Typography } from "@mui/material";

function GazeboViewer(props) {
  const { enableGazebo, handleEnableGazebo } = React.useContext(props.context);
  React.useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "ready") {
        handleEnableGazebo();
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
    <>
      {enableGazebo ? (
        <iframe
          id={"iframe"}
          style={{
            width: "100%",
            height: "400px",
          }}
          src={"http://127.0.0.1:6080/vnc.html?resize=remote&autoconnect=true"}
        />
      ) : (
        <Box
          sx={{
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
            border: "2px solid #d3d3d3",
            width: "100%",
            height: "400px",
            textAlign: "center",
            gap: "15px",
          }}
        >
          <Typography>{"Loading simulation   "} </Typography>
          <CircularProgress size={20} />
        </Box>
      )}
    </>
  );
}
GazeboViewer.propTypes = {
  context: PropTypes.any,
};

export default GazeboViewer;
