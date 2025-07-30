import React, { useEffect, useState } from "react";
import MenuItem from "@mui/material/MenuItem";
import { FormControl, InputLabel, Select, Box } from "@mui/material";
import {merge} from "lodash";

interface World {
  tools_config: Record<string, any>;
  [key: string]: any;
}

interface Config {
  name: string;
  tools_config: Record<string, any>;
  world: World;
  robot: any;
  tools: any[];
}

declare const context: {
  mapSelected: string;
};

interface WorldSelectorProps {

}

const WorldSelector: React.FC<WorldSelectorProps> = () => {
  const exerciseConfig: Config[] = JSON.parse(
    document.getElementById("exercise-config")?.textContent ?? "[]"
  );
  const [disabled, setDisabled] = useState<boolean>(true);
  const [selectedUniverse, setSelectedUniverse] = useState<Config>(exerciseConfig[0]);
  const [configurations, setConfigurations] = useState<Config[]>(exerciseConfig);

  useEffect(() => {
    context.mapSelected = exerciseConfig[0]?.name ?? "";

    const callback = (message: { data: { state: string } }) => {
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

  const handleUniverse = async (config: Config) => {
    context.mapSelected = config.name;
    setSelectedUniverse(config);

    await window.RoboticsExerciseComponents.commsManager.terminate_application();
    await window.RoboticsExerciseComponents.commsManager.terminate_tools();
    await window.RoboticsExerciseComponents.commsManager.terminate_universe();
    window.RoboticsReactComponents.MessageSystem.Loading.showLoading(
      "Launching Universe"
    );
    
    const tools_config = merge(config.tools_config, config.world.tools_config);

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
          onChange={(e: SelectChangeEvent<unknown>) => {
            const selectedConfig = e.target.value as Config;
            handleUniverse(selectedConfig);
          }}
          renderValue={(selected) => (selected as Config).name}
        >
          {configurations.map((option) => (
            <MenuItem key={option.name} value={option.name}>
              {option.name}
            </MenuItem>
          ))}
        </Select>
      </FormControl>
    </Box>
  ) : null;
}
