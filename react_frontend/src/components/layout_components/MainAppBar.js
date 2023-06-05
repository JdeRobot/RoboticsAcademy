import * as React from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import Image from "mui-image";
import { Box, Typography } from "@mui/material";
import RoboticsTheme from "Components/RoboticsTheme.js";
import PropTypes from "prop-types";
import { ConnectionIndicator } from "Components/visualizers/ConnectionIndicator";
import { LaunchIndicator } from "Components/visualizers/LaunchIndicator";
import { useUnload } from "Hooks/useUnload";

function MainAppBar(props) {
  const config = JSON.parse(
    document.getElementById("exercise-config").textContent
  );

  config.height = window.innerHeight / 2;
  config.width = window.innerWidth / 2;

  const connect = () => {
    window.RoboticsReactComponents.MessageSystem.Loading.showLoading(
      "Conectando y lanzando el ejercicio"
    );

    window.RoboticsExerciseComponents.commsManager
      .connect()
      .then(() => {
        window.RoboticsExerciseComponents.commsManager
          .launch(config)
          .then(() => {
            RoboticsReactComponents.MessageSystem.Loading.hideLoading();
            RoboticsReactComponents.MessageSystem.Alert.showAlert(
              "Ejercicio cargado correctamente"
            );
          })
          .catch((e) => {
            RoboticsReactComponents.MessageSystem.Alert.showAlert(
              e.data.message
            );
          });
      })
      .catch((e) => {
        RoboticsReactComponents.MessageSystem.Alert.showAlert(
          "Error conectando, prueba a recargar la página",
          () => {
            console.log("Reloading");
            window.location.reload();
          },
          "RECARGAR"
        );
      });
  };

  const disconnect = () => {
    window.RoboticsExerciseComponents.commsManager.disconnect();
  };

  React.useEffect(() => {
    RoboticsExerciseComponents.suscribeOnLoad(() => {
      connect();
    });
  }, []);

  useUnload(() => {
    disconnect();
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
