import * as React from "react";
import PropTypes from "prop-types";
import { Box } from "@mui/system";
import { CircularProgress, Typography } from "@mui/material";

function GazeboViewer(props) {
  const [enableGazebo, handleEnableGazebo] = React.useState(false);
  React.useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "visualization_ready") {
        handleEnableGazebo(true);
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
        <Box
          sx={{
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
            maxHeight: "100%",
            width: "100%",
            height: "100%",
            textAlign: "center",
          }}
        >
          <iframe
            id={"iframe"}
            style={{
              width: "100%",
              height: "100%",
            }}
            src={
              "http://127.0.0.1:6080/vnc.html?resize=remote&autoconnect=true&reconnect=true"
            }
          />
        </Box>
      ) : (
        <Box
          sx={{
            display: "flex",
            justifyContent: "center",
            alignItems: "center",

            width: "100%",
            height: "100%",
            textAlign: "center",
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
