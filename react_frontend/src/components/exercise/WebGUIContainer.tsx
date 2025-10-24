import { Box } from "@mui/system";
import React, { ReactNode } from "react";

const WebGUIContainer = ({
  id,
  children,
}: {
  id?: string;
  children?: ReactNode;
}) => {
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
      }}
    >
      {children}
    </Box>
  );
};

export default WebGUIContainer;
