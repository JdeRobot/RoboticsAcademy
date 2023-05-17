import * as React from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import IconButton from "@mui/material/IconButton";
import HomeIcon from "@mui/icons-material/Home";
import { Box, ButtonGroup, Typography } from "@mui/material";
import SchoolOutlinedIcon from "@mui/icons-material/SchoolOutlined";
import CodeOutlinedIcon from "@mui/icons-material/CodeOutlined";
import CommentOutlinedIcon from "@mui/icons-material/CommentOutlined";
import ViewContext from "../../contexts/ViewContext";
import RoboticsTheme from "../RoboticsTheme.js";
import PropTypes from "prop-types";
import { ConnectionIndicator } from "./RAM/ConnectionIndicator";
import { LaunchIndicator } from "./RAM/LaunchIndicator";

const serverBase = `${document.location.protocol}//${document.location.hostname}:8000`;

function MainAppBar(props) {
  const {
    theoryMode,
    codeMode,
    forumMode,
    openTheory,
    openExercise,
    openForum,
  } = React.useContext(ViewContext);

  const retryInterval = 1000;

  let ros_version = 1;

  const fetchRosVersion = (data) => {
    // Requests ROS version and filters exercises by ROS tag
    const rosVersionURL = `${serverBase}/exercises/ros_version/`;
    fetch(rosVersionURL)
        .then((res) => res.json())
        .then((msg) => {
          ros_version = msg.version;
          // If ROS is not installed
          if (isNaN(parseInt(ros_version))) {          
            ros_version = 1;
          }
        })
        .catch((error) => {
          ros_version = 1
        })
  };

  React.useEffect(() => {
    const connectRetry = setInterval(() => {
      fetchRosVersion();
      window.RoboticsExerciseComponents.commsManager
        .connect()
        .then(() => {
          console.log("Successfully connected");
          clearInterval(connectRetry);        

          const launchRetry = setInterval(() => {
            const config = JSON.parse(
              document.getElementById("exercise-config").textContent
            );
            // Selects the configs for the ROS installed
            const rosConfig = {};
            let key = "ROS" + ros_version;
            for (const [configKey, configValue] of Object.entries(config)) {
              if (configKey === key) {
                rosConfig[configKey] = configValue;
              }
            }            
            // Creates the config to send
            const launchConfig = {};
            // Compatibility, if there is no ROS data, send the complete object
            if (Object.keys(rosConfig).length == 0) {
              launchConfig = config;
            }
            for (const [configKey, configValue] of Object.entries(rosConfig[key])) {
              launchConfig[configKey] = configValue;
            }
            launchConfig['exercise_id'] = config['exercise_id'];            
            launchConfig.height = window.innerHeight / 2;
            launchConfig.width = window.innerWidth / 2;         
            console.log(launchConfig);   
            window.RoboticsExerciseComponents.commsManager
              .launch(launchConfig)
              .then(() => {
                console.log("Successfully launched");
                clearInterval(launchRetry);
              })
              .catch((error) => {
                console.error("Launch failed, retrying...", error);
              });
          }, retryInterval);
        })
        .catch((error) => {
          console.error("Connection failed, retrying...", error);
        });
    }, retryInterval);

    return () => {
      clearInterval(connectRetry);
    };
  }, []);
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
            {" "}
            <a href="https://jderobot.github.io/">
              <img src="/static/common/img/logo.gif" width={50} alt="" />
            </a>
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
              {props.specificConfiguration}
            </Box>
          </Box>
          <Typography variant="h5">{props.exerciseName}</Typography>
          <ButtonGroup color={"loading"} variant={"contained"}>
            <IconButton href="/exercises">
              <HomeIcon fontSize="small" />
            </IconButton>
            <IconButton
              component={"span"}
              id={"open-exercise"}
              onClick={openExercise}
              color={codeMode ? "success" : "secondary"}
            >
              <CodeOutlinedIcon />
            </IconButton>
            <IconButton
              component={"span"}
              id={"open-theory"}
              onClick={openTheory}
              color={theoryMode ? "success" : "secondary"}
            >
              <SchoolOutlinedIcon />
            </IconButton>
            <IconButton
              id={"open-forum"}
              onClick={openForum}
              component={"span"}
              color={forumMode ? "success" : "secondary"}
            >
              <CommentOutlinedIcon />
            </IconButton>
          </ButtonGroup>
        </Toolbar>
      </AppBar>
    </RoboticsTheme>
  );
}

MainAppBar.propTypes = {
  context: PropTypes.any,
  exerciseName: PropTypes.string,
  specificConfiguration: PropTypes.any,
};

export default MainAppBar;
