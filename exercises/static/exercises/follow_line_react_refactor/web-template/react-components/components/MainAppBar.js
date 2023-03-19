import * as React from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import Image from "mui-image";
import {Box, ButtonGroup, Typography} from "@mui/material";
import RoboticsTheme from "Components/RoboticsTheme.js";
import PropTypes from "prop-types";
import {ConnectionIndicator} from "Components/common/RAM/ConnectionIndicator";
import {LaunchIndicator} from "Components/common/RAM/LaunchIndicator";
import {useLoad} from "Hooks/useLoad";
import {useUnload} from "Hooks/useUnload";

function MainAppBar(props) {
  const config = JSON.parse(
    document.getElementById("exercise-config").textContent
  );

  config.height = window.innerHeight / 2;
  config.width = window.innerWidth / 2;

  useLoad(() => {
    // window.RoboticsExerciseComponents.commsManager.connect().then(() => {
    //   window.RoboticsExerciseComponents.commsManager.launch(config);
    // });
  });

  useUnload(() => {
    window.RoboticsExerciseComponents.commsManager.disconnect();
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
            <Image src="/static/common/img/logo.gif" fit={"cover"} width={50}/>
            <Box
              sx={{
                display: "flex",
                gap: "10px",
                marginLeft: "10px",
                alignItems: "center",
                justifyContent: "center",
              }}
            >
              <ConnectionIndicator></ConnectionIndicator>
              <LaunchIndicator></LaunchIndicator>
            </Box>
          </Box>
          <Typography variant="h5">{props.exerciseName}</Typography>
          {props.children}
        </Toolbar>
      </AppBar>
    </RoboticsTheme>
  );
}

MainAppBar.propTypes = {
  exerciseName: PropTypes.string,
};

export default MainAppBar;
