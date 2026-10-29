import BuildRoundedIcon from '@mui/icons-material/BuildRounded';
import { sendExecutionProbe } from "Api";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { CommsManager, states } from "jderobot-commsmanager";
import { useError } from "jderobot-ide-interface";
import { useState } from "react";
import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
const TerminateUniverseButton = ({ project }: { project: string }) => {
  const theme = useAcademyTheme();
  const { warning } = useError();
  const [loading, setLoading] = useState<boolean>(false);

  const terminateUniverse = async () => {
    const manager = CommsManager.getInstance();

    if (manager.getWorld() === undefined || manager.getWorld() === "") {
      return;
    }

    const state = manager.getState();

    if (state === states.IDLE || state === states.CONNECTED) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure a world is selected."
      );
      return;
    }

    sendExecutionProbe(project, "stop_world");

    setLoading(true);

    if (state === states.RUNNING || state === states.PAUSED) {
      await manager.terminateApplication();
      await manager.terminateTools();
      await manager.terminateWorld();
    } else if (state === states.TOOLS_READY) {
      await manager.terminateTools();
      await manager.terminateWorld();
    } else {
      await manager.terminateWorld();
    }
    setLoading(false);
  };

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="switch-tool"
      onClick={terminateUniverse}
      title="Switch tool"
      disabled={loading}
    >
      <BuildRoundedIcon htmlColor={theme.palette.text} />
    </StyledHeaderButton>
  );
};

export default TerminateUniverseButton;
