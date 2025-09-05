import { StyledHeaderButton } from "Components/headers/HeaderMenu.styles";
import { useError } from "jderobot-ide-interface";
import { CommsManager } from "jderobot-commsmanager";
import ReplayRoundedIcon from "@mui/icons-material/ReplayRounded";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";

const ResetButton = ({
  manager,
  setAppRunning,
}: {
  manager: CommsManager | null;
  setAppRunning: Function;
}) => {
  const theme = useAcademyTheme();
  const { warning } = useError();

  const onResetApp = async () => {
    if (!manager) {
      console.error("Manager is not running");
      warning(
        "Failed to connect with the Robotics Backend docker. Please make sure it is connected."
      );
      return;
    }

    if (
      manager.getState() !== "tools_ready" &&
      manager.getState() !== "application_running" &&
      manager.getState() !== "paused"
    ) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure an universe is selected."
      );
      return;
    }

    await manager.terminateApplication();
    console.log("App reseted!");
    setAppRunning(false);
  };


  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="reset-app"
      onClick={onResetApp}
      title="Reset app"
    >
      <ReplayRoundedIcon htmlColor={theme.palette.text} />
    </StyledHeaderButton>
  );
};

export default ResetButton;
