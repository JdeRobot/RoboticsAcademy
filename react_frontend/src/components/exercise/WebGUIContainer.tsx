import { Box } from "@mui/system";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import React, { ReactNode } from "react";

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
        position: "relative"
      }}
    >
      {children}
    </Box>
  );
};

export default WebGUIContainer;
