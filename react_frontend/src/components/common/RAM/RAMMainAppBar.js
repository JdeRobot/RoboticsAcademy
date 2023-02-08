import React, { useState } from "react";
import PropTypes from "prop-types";
import { useLoad } from "../../../hooks/useLoad";
import { useUnload } from "../../../hooks/useUnload";
import RoboticsTheme from "../../RoboticsTheme";
import { AppBar, Toolbar, Typography } from "@mui/material";
import { Box } from "@mui/system";
import Image from "mui-image";
import { ConnectionIndicator } from "./ConnectionIndicator";
import { LaunchIndicator } from "./LaunchIndicator";

export const RAMMainAppBar = (props) => {
  const config = JSON.parse(
    document.getElementById("exercise-config").textContent
  );
  config.height = window.innerHeight / 2;
  config.width = window.innerWidth / 2;

  useLoad(() => {
    window.RoboticsExerciseComponents.commsManager.connect().then(() => {
      window.RoboticsExerciseComponents.commsManager.launch(config);
      console.log("connected");
    });
  });

  useUnload(() => {
    window.RoboticsExerciseComponents.commsManager.terminate().then(() => {
      console.log("terminated");
    });
  });
  return (
    <RoboticsTheme>
      <AppBar position="static">
        <Toolbar
          sx={{
            display: "flex",
            flexWrap: "wrap",
            justifyContent: "space-between",
          }}
        >
          <Box
            sx={{
              display: "inline-flex",
              justifyContent: "space-between",
              alignItems: "center",
            }}
          >
            <Image src="/static/common/img/logo.gif" fit={"cover"} width={50} />
            <ConnectionIndicator></ConnectionIndicator>
            <LaunchIndicator></LaunchIndicator>
          </Box>
          <Typography variant="h5">{props.exerciseName}</Typography>
        </Toolbar>
      </AppBar>
    </RoboticsTheme>
  );
};

RAMMainAppBar.propTypes = {
  exerciseName: PropTypes.string,
};
