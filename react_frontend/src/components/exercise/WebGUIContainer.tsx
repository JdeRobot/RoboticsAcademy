import { Box } from "@mui/system";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { CommsManager } from "jderobot-commsmanager";
import React, { ReactNode, useRef } from "react";

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

export const connectApplication = () => {
  const ref = useRef<NodeJS.Timer>();
  const start = (manager: CommsManager) => {
    end()

    if (manager.ws.readyState !== WebSocket.OPEN) {
      return
    }
    
    if (ref.current === undefined) {
      ref.current = setInterval(() => {
        try {
          manager.send("gui", "start");
        } catch {
          end()
        }
      }, 1000);
    }
  };
  const end = () => {
    if (ref.current !== undefined) {
      clearInterval(ref.current);
      ref.current = undefined;
    }
  };
  return {start:start, end:end};
};

export default WebGUIContainer;
