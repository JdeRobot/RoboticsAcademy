import { StyledHeaderButton } from "Components/headers/HeaderMenu.styles";
import { useError, useTheme } from "jderobot-ide-interface";
import { CommsManager, states } from "jderobot-commsmanager";
import StopCircleRoundedIcon from "@mui/icons-material/StopCircleRounded";

const TerminateUniverseButton = ({
  manager,
  setAppRunning,
}: {
  manager: CommsManager | null;
  setAppRunning: Function;
}) => {
  const theme = useTheme();
  const { warning } = useError();

  const terminateUniverse = async () => {
    if (!manager) {
      console.error("Manager is not running");
      warning(
        "Failed to connect with the Robotics Backend docker. Please make sure it is connected."
      );
      return;
    }

    if (manager.getUniverse() === "") {
      return;
    }

    const state = manager.getState();

    if (state === states.IDLE || state === states.CONNECTED) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure an universe is selected."
      );
      return;
    }

    if (state === states.RUNNING || state === states.PAUSED) {
      await manager.terminateApplication();
      setAppRunning(false);
    }

    if (manager.getState() === states.TOOLS_READY) {
      await manager.terminateTools();
    }

    if (manager.getState() === states.WORLD_READY) {
      await manager.terminateUniverse();
    }
  };

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="stop-universe"
      onClick={terminateUniverse}
      title="Stop Universe"
    >
      <StopCircleRoundedIcon htmlColor={theme.palette.text} />
    </StyledHeaderButton>
  );
};

export default TerminateUniverseButton;
