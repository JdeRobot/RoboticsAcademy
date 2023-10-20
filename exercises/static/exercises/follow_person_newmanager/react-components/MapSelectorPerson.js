import React, { useEffect, useState } from "react";
import MenuItem from "@mui/material/MenuItem";
import { FormControl, InputLabel, Select, Box } from "@mui/material";

const exerciseConfig = JSON.parse(
  document.getElementById("exercise-config").textContent
);
const exerciseId = exerciseConfig.exercise_id;

export default function MapSelectorFollow(props) {

  const handleCircuitChange = (e) => {
    context.mapSelected = e.launch["0"].name
    setSelectedCircuit(e);
    const config = e;
    config['exercise_id'] = exerciseId;
    config["visualization"] = "gazebo_rae"
    config.height = window.innerHeight / 2;
    config.width = window.innerWidth / 2;         
    window.RoboticsExerciseComponents.commsManager.terminate().then(() => {
      window.RoboticsReactComponents.MessageSystem.Loading.showLoading(
        "Launching World in Robotics Backend"
      );
      window.RoboticsExerciseComponents.commsManager.launch(config).then(()=> {
        RoboticsReactComponents.MessageSystem.Loading.hideLoading();
      }).catch((e) => {
        RoboticsReactComponents.MessageSystem.Loading.showFailLoading(
          `Error launching the world:${e.data.message}. Try changing the world or reloading the page`
        );
      });
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
    const serverBase = `${document.location.protocol}//${document.location.hostname}:7164`;
    let requestUrl = `${serverBase}/exercises/exercise/${exerciseId}/launch_files`;
    const request = new Request(requestUrl, {
      method: "POST",
      headers: {
        "Content-type": "application/json",
        'X-CSRFToken': context.csrf
      },
    })
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
            setCircuitOptions(availableConfigs[`ROS${ros_version}`])
            context.mapSelected =
              availableConfigs[`ROS${ros_version}`][0].launch["0"].name;
          })
            setCircuitOptions(data.launch);   
                
        })
        .catch((error) => {
            console.log("Error fetching circuit options:", error);
        });
  }, []);

  

  return (
    <Box >
      <FormControl   sx={{
          m: 1,
          minWidth: 120,
          backgroundColor: disabled ?  "#f57f51":"#4caf50" ,
          border: "solid 0.4px black",
          borderRadius: "5px"
        }} size="small">
        <InputLabel id={"circuit-selector-label"}>
          World
        </InputLabel>
        <Select
          
          value={selectedCircuit}
          labelId="circuit-selector-label"
          id={"circuit-selector"}
          label={"Circuit"}
          onChange={(e) => {
            setSelectedCircuit(e.target.value);
            handleCircuitChange(e.target.value);
          }}
        >
          {circuitOptions && circuitOptions.map((option) => (
            <MenuItem key={option.launch["0"].name} value={option}>
              {option.launch["0"].name}
            </MenuItem>
          ))}
        </Select>
      </FormControl>
    </Box>
  );
}
