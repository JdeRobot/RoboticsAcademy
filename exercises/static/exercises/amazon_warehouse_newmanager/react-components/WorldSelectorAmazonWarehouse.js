import React, { useEffect, useState } from "react";
import MenuItem from "@mui/material/MenuItem";
import { FormControl, InputLabel, Select, Box } from "@mui/material";


const exerciseConfig = JSON.parse(
  document.getElementById("exercise-config").textContent
);
const exerciseId = "amazon_warehouse_newmanager";

export default function WorldSelectorAmazonWarehouse(props) {
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
        if (config.name == "World 2") {
            document.getElementById("amazon_map_canvas").style.backgroundImage = "url('/static/exercises/amazon_warehouse_newmanager/resources/images/map_2.png')";
          }
          else {
            document.getElementById("amazon_map_canvas").style.backgroundImage = "url('/static/exercises/amazon_warehouse_newmanager/resources/images/map.png')";
          }
        setSelectedCircuit(config);
        window.RoboticsExerciseComponents.commsManager.terminate().then(() => {
          window.RoboticsReactComponents.MessageSystem.Loading.showLoading(
            "Launching World"
          );
          window.RoboticsExerciseComponents.commsManager
            .launch(config)
            .then(() => {
              RoboticsReactComponents.MessageSystem.Loading.hideLoading();
            })
            .catch((e) => {
              RoboticsReactComponents.MessageSystem.Loading.showFailLoading(
                `Error launching the world:${e.data.message}. Try changing the world or reloading the page`
              );
            });
        });
      };
    
      return exerciseConfig.length > 0 ? (
        <Box>
          <FormControl
            sx={{
              m: 1,
              minWidth: 120,
              backgroundColor: disabled ? "#f57f51" : "#4caf50",
            }}
            size="small"
          >
            <InputLabel id={"circuit-selector-label"}>World</InputLabel>
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
