import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
import { useError } from "jderobot-ide-interface";
import { CommsManager, states } from "jderobot-commsmanager";
import ReplayRoundedIcon from "@mui/icons-material/ReplayRounded";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import React, { useEffect, useState } from "react";
import { subscribe, unsubscribe } from "Helpers/utils";
import SyncRoundedIcon from "@mui/icons-material/SyncRounded";

const ResetButton = () => {
  const theme = useAcademyTheme();
  const { warning, error } = useError();

  const [state, setState] = useState<string>(
    CommsManager.getInstance().getState()
  );
  const [loading, setLoading] = useState<boolean>(false);

  const updateState = (e: any) => {
    setState(e.detail.state);
  };

  useEffect(() => {
    subscribe("CommsManagerStateChange", updateState);

    return () => {
      unsubscribe("CommsManagerStateChange", () => {});
    };
  }, []);

  useEffect(() => {
    if (state === states.TOOLS_READY) {
      setLoading(false);
    }
  }, [state]);

  const onResetApp = async () => {
    const manager = CommsManager.getInstance();

    if (
      state === states.CONNECTED ||
      state === states.IDLE ||
      state === states.WORLD_READY
    ) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure an universe is selected."
      );
      return;
    }

    if (state === states.TOOLS_READY) {
      return;
    }

    setLoading(true);
    try {
      await manager.terminateApplication();
    } catch (e) {
      error("Failed to reset the application. See the traces in the terminal.");
    }
    console.log("App reseted!");
  };

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="reset-app"
      onClick={onResetApp}
      title="Reset app"
      disabled={loading}
    >
      {loading ? (
        <SyncRoundedIcon htmlColor={theme.palette.text} id="loading-spin" />
      ) : (
        <ReplayRoundedIcon htmlColor={theme.palette.text} />
      )}
    </StyledHeaderButton>
  );
};

export default ResetButton;
