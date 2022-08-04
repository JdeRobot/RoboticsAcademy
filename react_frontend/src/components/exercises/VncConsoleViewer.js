import * as React from "react";
import { Box, Typography } from "@mui/material";
import ExerciseContext from "../../contexts/ExerciseContext";

function VncConsoleViewer() {
  const { openConsole } = React.useContext(ExerciseContext);
  return (
    <Box display={openConsole ? "block" : "none"}>
      <Typography color={"secondary"} borderBottom={2} m={0.4}>
        Console
      </Typography>
      <iframe
        id={"console-vnc"}
        style={{
          width: "40vw",
          height: "50vh",
        }}
        src={
          openConsole
            ? "http://127.0.0.1:1108/vnc.html?resize=remote&autoconnect=true"
            : ""
        }
      />
    </Box>
  );
}

export default VncConsoleViewer;
