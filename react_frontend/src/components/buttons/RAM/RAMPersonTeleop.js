import { Button } from "@mui/material";
import React, { useContext, useEffect, useState } from "react";
import PropTypes from "prop-types";
import VideogameAssetOutlinedIcon from "@mui/icons-material/VideogameAssetOutlined";

const teleopHost = window.location.hostname;
const teleopPort = 7164;
const address = `ws://${teleopHost}:${teleopPort}`;

let teleOpMode = false;
let websocket;
let websocket_connected = false; 

const PersonTeleop = () => {
  const [disabled, setDisabled] = useState(true);

  useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "running") {
        setDisabled(false);
        if (!websocket_connected) {
          open_websocket_connection();
          websocket_connected = true;
        }
      } else {
        setDisabled(true);
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

  function open_websocket_connection() {

    websocket = new WebSocket(address);

    websocket.onopen = function(e) {
      console.log(`Connection with ${address} opened`);
    };

    websocket.onmessage = function(e) {
      console.log('Mensaje recibido: %s', e.data);
    };

    websocket.onclose = function(e) {
      if (e.wasClean) {
        console.log(`Connection with ${address} closed, all suscribers cleared`);
      } else {
        console.log(`Connection with ${address} interrupted`);
      }
    };

    websocket.onerror = function(e) {
      console.log(`Error received from websocket: ${e.type}`);
    };
  }

  function listen_key() {
    window.addEventListener("keypress", function (event) {
      if (teleOpMode) {
        if (event.code === "KeyS") {
          websocket.send("#key_s");
        } else if (event.code === "KeyW") {
          websocket.send("#key_w");
        } else if (event.code === "KeyA") {
          websocket.send("#key_a");
        } else if (event.code === "KeyD") {
          websocket.send("#key_d");
        } else if (event.code === "KeyX") {
          websocket.send("#key_x");
        }
      }
    });
  };

  function teleOpButtonClick() {
    if (!teleOpMode) {
      teleOpMode = true;
      websocket.send("#teleop_true");
    } else {
      teleOpMode = false;
      websocket.send("#teleop_false");
    }

    console.log("Teleoperate clicked, mode=%s", teleOpMode);
    listen_key();
  }

  return (
    <Button
      id={"teleop_button"}
      size={"medium"}
      variant="contained"
      color={"secondary"}
      component="span"
      sx={{ m: 1 }}
      onClick={() => {
        teleOpButtonClick();
      }}
      title={"Use the arrow keys to operate the person"}
      disabled={disabled}
      startIcon={<VideogameAssetOutlinedIcon />}
    >
      Teleoperate
    </Button>
  );
};
PersonTeleop.propTypes = {
  context: PropTypes.any,
};
export default PersonTeleop;
