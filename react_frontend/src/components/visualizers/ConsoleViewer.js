import * as React from "react";
import PropTypes from "prop-types";
import { Box } from "@mui/system";
import { CircularProgress, Typography } from "@mui/material";

function VncConsoleViewer() {
  const [active, setActive] = React.useState(false);
  React.useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "ready") {
        setTimeout(() => {
          setActive(true);
        }, 1000);
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
      {active ? (
        <Box className="gazebo-view" sx={{ height: "100%" }}>
          <iframe
            id={"iframe-console"}
            src={
              "http://127.0.0.1:1108/vnc.html?resize=remote&autoconnect=true"
            }
            style={{
              height: "100%",
              width: "100%",
            }}
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
          <Typography>{"Loading console   "} </Typography>
          <CircularProgress size={20} />
        </Box>
      )}
    </>
  );
}
VncConsoleViewer.propTypes = {
  context: PropTypes.any,
};

export default VncConsoleViewer;
