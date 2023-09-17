import React, { useEffect, useState } from "react";

import MenuItem from "@mui/material/MenuItem";
import LandscapeIcon from "@mui/icons-material/Landscape";

import { FormControl, InputLabel, Select, Box } from "@mui/material";

const serverBase = `${document.location.protocol}//${document.location.hostname}:7164`;
const exerciseConfig = JSON.parse(
  document.getElementById("exercise-config").textContent
);
const exerciseId = exerciseConfig.exercise_id;

export default function MapSelectorPerson(props) {
  const changeConfig = (circuitPath) => {
    const config = JSON.parse(
      document.getElementById("exercise-config").textContent
    );
    config.application.params = { circuit: circuitPath };
    config.launch[
      "0"
    ].launch_file = `$EXERCISE_FOLDER/launch/simple_line_follower_ros_headless_${circuitPath}.launch`;
    return config;
  };

  const handleCircuitChange = (e) => {
    const config = e;
    setSelectedCircuit(e);
    console.log(JSON.stringify(config));
    config['exercise_id'] = exerciseId;
    config.height = window.innerHeight / 2;
    config.width = window.innerWidth / 2;         
    window.RoboticsExerciseComponents.commsManager.terminate().then(() => {
      window.RoboticsExerciseComponents.commsManager.launch(config);
    });
  };

  const [disabled, setDisabled] = useState(true);
  const [circuitOptions, setCircuitOptions] = useState([]);
  const [selectedCircuit, setSelectedCircuit] = useState("");

  useEffect(() => {
    const callback = (message) => {
      if (message.data.state !== "connected") {
        setDisabled(false);
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

  useEffect(() => {
    const mapsAvailableURL = `${serverBase}/exercises/exercise/${exerciseId}/launch_files`;
    fetch(mapsAvailableURL)
      .then((response) => response.json())
      .then((data) => {    
        const rosVersionURL = `${serverBase}/exercises/ros_version/`;    
        let ros_version = 1;
        fetch(rosVersionURL)
        .then((res) => res.json())
        .then((msg) => {          
          ros_version = msg.version;
          // If returns no version, assume 1
          if (isNaN(parseInt(ros_version))) {          
            ros_version = 1;
          }
          const config = data;
          // Selects the configs available for the ROS version installed          
          const availableConfigs = {};
          if(ros_version === "2"){
            availableConfigs[`ROS${ros_version}`] = config[`ROS${ros_version}`];
            setCircuitOptions(availableConfigs[`ROS${ros_version}`]);
            setSelectedCircuit(availableConfigs[`ROS${ros_version}`][0]);
          }
                   
        })
        .catch((error) => {
          const availableConfigs = {};
          availableConfigs[`ROS${ros_version}`] = config[`ROS${ros_version}`];
          setCircuitOptions(availableConfigs[`ROS${ros_version}`]);
          setSelectedCircuit(availableConfigs[`ROS${ros_version}`][0]);
        })        
      })
      .catch((error) => {
        console.log("Error fetching circuit options:", error);
      });
  }, []);

  return circuitOptions.length > 0 ? (
    <Box sx={{ marginLeft: "20px" }}>
      <FormControl>
        <InputLabel id={"circuit-selector-label"}>
          <LandscapeIcon></LandscapeIcon>
        </InputLabel>
        <Select
          disabled={disabled}
          value={selectedCircuit}
          labelId="circuit-selector-label"
          id={"circuit-selector"}
          label={"Circuit"}
          onChange={(e) => {
            handleCircuitChange(e.target.value);
          }}
        >
          <MenuItem disabled value="1">
            Select world
          </MenuItem>
          {circuitOptions.map((option) => (
            <MenuItem key={option.launch["0"].name} value={option}>
              {option.launch["0"].name}
            </MenuItem>
          ))}
        </Select>
      </FormControl>
    </Box>
  ) : null;
}