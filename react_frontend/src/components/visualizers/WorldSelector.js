import React, { useEffect, useState } from "react";
import MenuItem from "@mui/material/MenuItem";
import { FormControl, InputLabel, Select, Box } from "@mui/material";

export default function WorldSelector(props) {
  const exerciseConfig = JSON.parse(
    document.getElementById("exercise-config").textContent
  );
  const [disabled, setDisabled] = useState(true);
  const [selectedCircuit, setSelectedCircuit] = useState(exerciseConfig[0]);
  const [configurations, setConfigurations] = useState(exerciseConfig);

  useEffect(() => {
    context.mapSelected = exerciseConfig[0].name;

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

  const handleCircuitChange = (config) => {
    
    context.mapSelected = config.name;
    setSelectedCircuit(config);
    console.log(config.visualization);
    window.RoboticsExerciseComponents.commsManager
      .terminate_application()
      .then(() => {
        window.RoboticsExerciseComponents.commsManager
        .terminate_visualization()
        .then(() => {
          window.RoboticsExerciseComponents.commsManager
          .terminate_universe()
          .then(() => {
            window.RoboticsReactComponents.MessageSystem.Loading.showLoading(
              "Launching Universe"
            );
            window.RoboticsExerciseComponents.commsManager
              .launchWorld(config)
              .then(() => {
                window.RoboticsExerciseComponents.commsManager
                .prepareVisualization(config.visualization)
                .then(() => {
                  RoboticsReactComponents.MessageSystem.Loading.hideLoading();
                  RoboticsReactComponents.MessageSystem.Alert.showAlert(
                    "Exercise loaded successfully.", "success"
                  );
                })
         
              })
          });
        })
      })
  };

  return exerciseConfig.length > 0 ? (
    <Box>
      <FormControl
        sx={{
          m: 1,
          minWidth: 120,
          maxWidth: 150,
          backgroundColor: disabled ? "#f57f51" : "#4caf50",
          textOverFlow: "clip",
        }}
        size="small"
      >
        <InputLabel id={"circuit-selector-label"}>Universe</InputLabel>
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
          {configurations.map((option) => (
            <MenuItem key={option.name} value={option}>
              {option.name}
            </MenuItem>
          ))}
        </Select>
      </FormControl>
    </Box>
  ) : null;
}
