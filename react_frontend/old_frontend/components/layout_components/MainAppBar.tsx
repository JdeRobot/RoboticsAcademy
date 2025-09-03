import * as React from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import Image from "mui-image";
import { Box } from "@mui/material";
import RoboticsTheme from "Components/RoboticsTheme";
import PropTypes from "prop-types";
import { useUnload } from "Hooks/useUnload";
import ExerciseTheoryForumButton from "../buttons/ExerciseTheoryForumButton";
import AppIndicator from "../visualizers/AppIndicator";
import ConnectionIndicator from "../visualizers/ConnectionIndicator";
import {VisualizationIndicator} from "../visualizers/VisualizationIndicator";
import {merge} from "lodash";

interface MainAppBarProps {
  exerciseName: string;
  url: string;
  children?: React.ReactNode;
}

const MainAppBar: React.FC<MainAppBarProps> = ({ exerciseName, url, children }) => {
  const maxConnectionAttempts = 3;
  let connectionAttempts = 0;

  const connectWithRetry = (): void => {
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
        var tools_config_base = config[0].tools_config
        var tools_config_world = config[0].world.tools_config
        const tools_config = merge(tools_config_base, tools_config_world)

        window.RoboticsExerciseComponents.commsManager
          .launchWorld({ world: config[0].world, robot: config[0].robot })
          .then(() => {
            window.RoboticsExerciseComponents.commsManager
              .prepareTools({ tools: config[0].tools, config: tools_config })
              .then(() => {
                RoboticsReactComponents.MessageSystem.Loading.hideLoading();
                RoboticsReactComponents.MessageSystem.Alert.showAlert(
                  "Exercise loaded successfully.",
                  "success"
                );
              });
          })
          .catch((e: any) => {
            RoboticsReactComponents.MessageSystem.Alert.showAlert(
              e?.data?.message ?? "Failed to launch world",
              "error"
            );
          });
      })
      .catch(() => {
        connectionAttempts++;
        setTimeout(connectWithRetry, 2000);
      });
  };

  const disconnect = (): void => {
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
              <img
                src="/static/exercises/assets/img/logo.gif"
                style={{ objectFit: "cover" }}
                width={50}
                alt="Robotics Academy Logo"
              />
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
            <ConnectionIndicator />
            {children}
            <VisualizationIndicator />
            <AppIndicator name={exerciseName} />
          </Box>
          <Box>
            <ExerciseTheoryForumButton url={url} />
          </Box>
        </Toolbar>
      </AppBar>
    </RoboticsTheme>
  );
};

export default MainAppBar;
