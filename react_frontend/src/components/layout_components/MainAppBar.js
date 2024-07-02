import * as React from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import Image from "mui-image";
import { Box } from "@mui/material";
import RoboticsTheme from "Components/RoboticsTheme.js";
import PropTypes from "prop-types";
import { useUnload } from "Hooks/useUnload";
import ExerciseTheoryForumButton from "../buttons/ExerciseTheoryForumButton";
import AppIndicator from "../visualizers/AppIndicator";
import ConnectionIndicator from "../visualizers/ConnectionIndicator";
import VisualizationIndicator from "../visualizers/VisualizationIndicator";

function MainAppBar(props) {
  const maxConnectionAttempts = 3;
  let connectionAttempts = 0;

  const connectWithRetry = () => {
    if (connectionAttempts >= maxConnectionAttempts) {
      RoboticsReactComponents.MessageSystem.Alert.showAlert(
        "Error connecting, try reloading the page.",
        "error",
        () => {
          console.log("Reloading");
          window.location.reload();
        },
        "RELOAD"
      );
      return;
    }

    window.RoboticsReactComponents.MessageSystem.Loading.showLoading(
      "Connecting and launching the exercise."
    );

    window.RoboticsExerciseComponents.commsManager
      .connect()
      .then(() => {
        const config = JSON.parse(
          document.getElementById("exercise-config").textContent
        );
        window.RoboticsExerciseComponents.commsManager
          .launchWorld(config[0])
          .then(() => {
            window.RoboticsExerciseComponents.commsManager
            .prepareVisualization(config[0].visualization).then(() => {
              RoboticsReactComponents.MessageSystem.Loading.hideLoading();
              RoboticsReactComponents.MessageSystem.Alert.showAlert(
                "Exercise loaded successfully.", "success"
              );
            })
     
          })
          .catch((e) => {
            RoboticsReactComponents.MessageSystem.Alert.showAlert(
              e.data.message, "error"
            );
          });
      })
      .catch((e) => {
        // Connection failed, try again after a delay
        connectionAttempts++;
        setTimeout(connectWithRetry, 2000);
      });
  };

  const disconnect = () => {
    window.RoboticsExerciseComponents.commsManager.disconnect();
  };

  React.useEffect(() => {
    RoboticsExerciseComponents.suscribeOnLoad(() => {
      connectWithRetry();
    });
  }, []);

  useUnload(() => {
    disconnect();
  });

  return (
    <RoboticsTheme>
      <AppBar position="relative">
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
            <a href="http://127.0.0.1:7164/exercises/">
              <img src="/static/exercises/assets/img/logo.gif" fit={"cover"} width={50} />
            </a>
          </Box>
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
            {props.children}
            <VisualizationIndicator></VisualizationIndicator>
            <AppIndicator name={props.exerciseName}></AppIndicator>
          </Box>

          <Box>
            <ExerciseTheoryForumButton
              url={props.url}
            ></ExerciseTheoryForumButton>
          </Box>
        </Toolbar>
      </AppBar>
    </RoboticsTheme>
  );
}

MainAppBar.propTypes = {
  exerciseName: PropTypes.string,
};

export default MainAppBar;
