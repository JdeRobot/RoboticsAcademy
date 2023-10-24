import React, { useEffect, useState } from "react";

import MenuItem from "@mui/material/MenuItem";
import LandscapeIcon from "@mui/icons-material/Landscape";

import { FormControl, InputLabel, Select, Box } from "@mui/material";

const serverBase = `${document.location.protocol}//${document.location.hostname}:7164`;
const exerciseConfig = JSON.parse(
  document.getElementById("exercise-config").textContent
);
const exerciseId = exerciseConfig.exercise_id;
var ros_version = 1;

export default function MapSelectorFollow(props) {
  const changeConfig = (circuitPath) => {
    const config = JSON.parse(
      document.getElementById("exercise-config").textContent
    );
    config.application.params = { circuit: circuitPath };
    config.launch_file = `$EXERCISE_FOLDER/launch/simple_line_follower_ros_headless_${circuitPath}.launch`;
    return config;
  };

  const handleCircuitChange = (e) => {
    context.mapSelected = e.name;
    setSelectedCircuit(e);
    let full_config = JSON.parse(
      document.getElementById("exercise-config").textContent
    );
    let config = full_config["ROS1"][0];        
    config.application.params = { circuit: e.name };
    config.launch_file = e.path;
    config['exercise_id'] = exerciseId;
    config["world"] = "gazebo";
    config["visualization"] = "gazebo_rae";
    config["world"] = "gazebo";
    config["resource_folders"] = "$EXERCISE_FOLDER/launch/ros1_noetic";
    config["model_folders"] = "$CUSTOM_ROBOTS_FOLDER/f1/models";
    config["visualization"] = "gazebo_rae";
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
        ros_version = 1;
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
          availableConfigs[`ROS${ros_version}`] = config[`ROS${ros_version}`];
          setCircuitOptions(availableConfigs[`ROS${ros_version}`]);   
          setSelectedCircuit(availableConfigs[`ROS${ros_version}`][0].name); 
          context.mapSelected = availableConfigs[`ROS${ros_version}`][0].name;
          setSelectedCircuit("Simple");
        })
        .catch((error) => {
          const availableConfigs = {};
          availableConfigs = config[`ROS${ros_version}`];
          setCircuitOptions(availableConfigs);
          setSelectedCircuit(availableConfigs[`ROS${ros_version}`][0].name);
        })        
      })
      .catch((error) => {
        console.log("Error fetching circuit options:", error);
      });
      setSelectedCircuit("Simple");
  }, []);

  

  return (

    circuitOptions ? 
    <Box sx={{marginLeft: "20px"}}>
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
            setSelectedCircuit(e.target.value);
            handleCircuitChange(e.target.value);
          }}
        >
          {circuitOptions.map((option) => (
            <MenuItem key={option.name} value={option}>
              {option.name}
            </MenuItem>
          ))}
        </Select>
      </FormControl>
    </Box> : null
  );
}
