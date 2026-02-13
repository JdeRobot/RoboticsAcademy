import { useState } from "react";
import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
import { useError } from "jderobot-ide-interface";
import { CommsManager, states } from "jderobot-commsmanager";
import StopCircleRoundedIcon from "@mui/icons-material/StopCircleRounded";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import SyncRoundedIcon from "@mui/icons-material/SyncRounded";

const TerminateUniverseButton = () => {
  const theme = useAcademyTheme();
  const { warning } = useError();
  const [loading, setLoading] = useState<boolean>(false);

  const terminateUniverse = async () => {
    const manager = CommsManager.getInstance();

    if (manager.getUniverse() === undefined || manager.getUniverse() === "") {
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

    setLoading(true);

    if (state === states.RUNNING || state === states.PAUSED) {
      await manager.terminateApplication();
      await manager.terminateTools();
      await manager.terminateUniverse();
    } else if (state === states.TOOLS_READY) {
      await manager.terminateTools();
      await manager.terminateUniverse();
    } else {
      await manager.terminateUniverse();
    }

    setLoading(false);
  };

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="stop-universe"
      onClick={terminateUniverse}
      title="Stop Universe"
      disabled={loading}
    >
      {loading ? (
        <SyncRoundedIcon htmlColor={theme.palette.text} id="loading-spin" />
      ) : (
        <StopCircleRoundedIcon htmlColor={theme.palette.text} />
      )}
    </StyledHeaderButton>
  );
};

export default TerminateUniverseButton;
