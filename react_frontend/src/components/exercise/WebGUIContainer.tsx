import { Box } from "@mui/system";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import {
  CommsManager,
  events,
  ManagerMsg,
  states,
} from "jderobot-commsmanager";
import React, { MutableRefObject, ReactNode, useEffect, useRef } from "react";

const WebGUIContainer = ({
  children,
}: {
  children?: ReactNode;
}) => {
  const theme = useAcademyTheme();

  return (
    <Box
      id="webgui-container"
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

export const connectApplication = (
  manager: CommsManager | null,
  updateCallback: (data: unknown) => void,
  stateCallback?: (state: string) => void,
  resizeRef?: MutableRefObject<HTMLImageElement | null>,
  resizeObserver?: ResizeObserver
) => {
  const intRef = useRef<number | null>(null);

  const onStateChange = (message: ManagerMsg) => {
    const state = message.data.state as string;
    if (state === states.TOOLS_READY || state === states.RUNNING) {
      start();
    }
    if (stateCallback !== undefined) {
      stateCallback(state);
    }
  };

  const onUpdate = (message: ManagerMsg) => {
    if (manager === null) {
      return;
    }

    end();
    updateCallback(message.data);
    manager.send("gui", "ack"); // Send the ACK of the msg
  };

  useEffect(() => {
    if (manager === null) {
      return;
    }

    if (resizeRef && resizeRef.current && resizeObserver) {
      resizeObserver.observe(resizeRef.current);
    }

    manager.subscribe(events.UPDATE, onUpdate);
    manager.subscribe(events.STATE_CHANGED, onStateChange);

    return () => {
      manager.unsubscribe(events.UPDATE, onUpdate);
      manager.unsubscribe(events.STATE_CHANGED, onStateChange);
    };
  }, [manager]);

  const start = () => {
    if (manager === null) {
      return;
    }

    end();

    if (manager.ws.readyState !== WebSocket.OPEN) {
      return;
    }

    const timer = window.setInterval(async () => {
      try {
        await manager.send("gui", "start");
      } catch {
        window.clearInterval(timer);
      }
    }, 1000);
    intRef.current = timer;
  };

  const end = () => {
    if (intRef.current !== null) {
      window.clearInterval(intRef.current);
      intRef.current = null;
    }
  };
};

export default WebGUIContainer;
