import React, { useEffect, useState } from "react";
import MenuItem from "@mui/material/MenuItem";
import { FormControl, InputLabel, Select, Box } from "@mui/material";
import {merge} from "lodash";

export default function WorldSelector(props) {
  const exerciseConfig = JSON.parse(
    document.getElementById("exercise-config").textContent
  );
  const [disabled, setDisabled] = useState(true);
  const [selectedUniverse, setSelectedUniverse] = useState(exerciseConfig[0]);
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

  const handleUniverse = async (config) => {
    context.mapSelected = config.name;
    setSelectedUniverse(config);
    console.log(config);
    var tools = config.tools;
    tools.push("web_gui");

    await window.RoboticsExerciseComponents.commsManager.terminate_application();
    await window.RoboticsExerciseComponents.commsManager.terminate_tools();
    await window.RoboticsExerciseComponents.commsManager.terminate_universe();
    window.RoboticsReactComponents.MessageSystem.Loading.showLoading(
      "Launching Universe"
    );
    //TODO: add her correct
    var tools_config_base = config[0].tools_config
    var tools_config_world = config[0].world.tools_config
    const tools_config = merge(tools_config_base, tools_config_world)

    await window.RoboticsExerciseComponents.commsManager.launchWorld({
      world: config.world,
      robot: config.robot,
    });
    await window.RoboticsExerciseComponents.commsManager.prepareTools({
      tools: config.tools,
      config: tools_config,
    });
    RoboticsReactComponents.MessageSystem.Loading.hideLoading();
    RoboticsReactComponents.MessageSystem.Alert.showAlert(
      "Exercise loaded successfully.",
      "success"
    );
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
        <InputLabel id={"universe-selector-label"}>Universe</InputLabel>
        <Select
          disabled={disabled}
          value={selectedUniverse}
          labelId="universe-selector-label"
          id={"universe-selector"}
          label={"Universe"}
          onChange={(e) => {
            handleUniverse(e.target.value);
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
