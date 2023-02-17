import * as React from "react";
import { Box, Typography } from "@mui/material";
import PropTypes from "prop-types";

function GazeboViewer() {
  return (
    <Box display={"block"}>
      <Typography color={"secondary"} borderBottom={2} m={0.4}>
        Gazebo
      </Typography>
      <iframe
        id={"iframe"}
        style={{
          width: "500px",
          height: "250px",
        }}
        src={"http://127.0.0.1:6080/vnc.html?resize=remote&autoconnect=true"}
      />
    </Box>
  );
}
GazeboViewer.propTypes = {
  context: PropTypes.any,
};

export default GazeboViewer;
