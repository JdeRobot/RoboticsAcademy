import React, { useEffect, useState } from "react";
import { Box, Typography, Tooltip } from "@mui/material";
import "../../styles/Indicator.css";

const exerciseConfig = JSON.parse(
  document.getElementById("exercise-config").textContent
);
const exerciseId = exerciseConfig.exercise_id;

function LaunchIndicator(props) {
  const [selectedCircuit, setSelectedCircuit] = useState("");
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "ready") {
        setConnected(true);
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

  useEffect(() => {
    console.log(exerciseId);
    const serverBase = `${document.location.protocol}//${document.location.hostname}:7164`;
    let requestUrl = `${serverBase}/exercises/exercise/${exerciseId}/launch_files`;
    const request = new Request(requestUrl, {
      method: "POST",
      headers: {
        "Content-type": "application/json",
        "X-CSRFToken": context.csrf,
      },
    });
    fetch(request)
      .then((response) => response.json())
      .then((data) => {
        const rosVersionURL = `${serverBase}/exercises/ros_version/`;
        let ros_version = 1;
        fetch(rosVersionURL)
          .then((res) => res.json())
          .then((msg) => {
            ros_version = msg.version;

            if (isNaN(parseInt(ros_version))) {
              ros_version = 1;
            }
            const config = data;
            // Selects the configs available for the ROS version installed
            const availableConfigs = {};
            availableConfigs[`ROS${ros_version}`] = config[`ROS${ros_version}`];
            console.log(availableConfigs);
            setSelectedCircuit(availableConfigs[`ROS${ros_version}`][0]);
            context.mapSelected =
              availableConfigs[`ROS${ros_version}`][0].launch["0"].name;
          })
          .catch((error) => {
            console.error(error);
          });
      })
      .catch((error) => {
        console.log("Error fetching circuit options:", error);
      });
  }, []);

  return (
    <Tooltip title="World Launched">
      <Box className={connected ? "ready" : "waiting"}>
        <p className="title">World</p>
        <Typography sx={{ fontSize: "0.8rem" }} className="word">
          {selectedCircuit && selectedCircuit.launch["0"].name}
        </Typography>
      </Box>
    </Tooltip>
  );
}

export default LaunchIndicator;
