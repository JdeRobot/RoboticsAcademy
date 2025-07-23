import React, { useEffect, useState } from "react";
import MenuItem from "@mui/material/MenuItem";
import { FormControl, InputLabel, Select, Box } from "@mui/material";

interface UniverseConfig {
  name: string;
  world: string;
  robot: string;
  visualization: string;
  visualization_config_path: string;
}

declare global {
  interface Window {
    RoboticsExerciseComponents: any;
    RoboticsReactComponents: any;
    context: any;
  }
}

const exerciseConfig: UniverseConfig[] = JSON.parse(
  document.getElementById("exercise-config")?.textContent || "[]"
);

export default function WorldSelector() {
  const [disabled, setDisabled] = useState(true);
  const [selectedUniverse, setSelectedUniverse] = useState<UniverseConfig | null>(
    exerciseConfig.length > 0 ? exerciseConfig[0] : null
  );
  const [configurations, setConfigurations] = useState<UniverseConfig[]>(exerciseConfig);

  useEffect(() => {
    window.context.mapSelected = selectedUniverse.name;

    const callback = (message: MessageEvent<any>) => {
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

  const handleUniverse = async (config: UniverseConfig) => {
    context.mapSelected = config.name;
    setSelectedUniverse(config);
    console.log(config);

    await window.RoboticsExerciseComponents.commsManager.terminate_application();
    await window.RoboticsExerciseComponents.commsManager.terminate_visualization();
    await window.RoboticsExerciseComponents.commsManager.terminate_universe();
    window.RoboticsReactComponents.MessageSystem.Loading.showLoading(
      "Launching Universe"
    );
    await window.RoboticsExerciseComponents.commsManager.launchWorld({
      world: config.world,
      robot: config.robot,
    });
    await window.RoboticsExerciseComponents.commsManager.prepareVisualization(
      {type: config.visualization, file: config.visualization_config_path}
    );
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
          value={selectedUniverse.name || ""}
          labelId="universe-selector-label"
          id={"universe-selector"}
          label={"Universe"}
          onChange={(e) => {
              const selected = configurations.find((c) => c.name === e.target.value);
              if (selected) {
                handleUniverse(selected);
              }
            }}
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
