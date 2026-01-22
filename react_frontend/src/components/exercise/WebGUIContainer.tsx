import { Box } from "@mui/system";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { CommsManager, events, states } from "jderobot-commsmanager";
import React, { ReactNode, useEffect, useRef } from "react";

const WebGUIContainer = ({
  id,
  children,
}: {
  id?: string;
  children?: ReactNode;
}) => {
  const theme = useAcademyTheme();

  return (
    <Box
      id={id}
      sx={{
        display: "flex",
        justifyContent: "center",
        alignItems: "center",
        maxHeight: "100%",
        width: "100%",
        height: "100%",
        textAlign: "center",
        backgroundColor: theme.palette.bg,
        position: "relative",
      }}
    >
      {children}
    </Box>
  );
};

export const connectApplication = (manager: CommsManager) => {
  const ref = useRef<NodeJS.Timer>();

  const onStateChange = (message: any) => {
    const state = message.data.state
    if (state === states.TOOLS_READY || state === states.RUNNING) {
      start();
    }
  };

  useEffect(() => {
    if (manager === null) {
      return;
    }

    manager.subscribe(events.STATE_CHANGED, onStateChange);

    return () => {
      manager.unsubscribe(events.STATE_CHANGED, onStateChange);
    };
  }, [manager]);

  const start = () => {
    end();

    if (manager.ws.readyState !== WebSocket.OPEN) {
      return;
    }

    ref.current = setInterval(() => {
      try {
        manager.send("gui", "start");
      } catch {
        end();
      }
    }, 1000);
  };

  const end = () => {
    if (ref.current !== undefined) {
      clearInterval(ref.current);
      ref.current = undefined;
    }
  };

  return {end: end};
};

export default WebGUIContainer;
