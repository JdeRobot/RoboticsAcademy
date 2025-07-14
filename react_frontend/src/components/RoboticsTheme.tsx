import React, { ReactNode } from "react";
import { createTheme, ThemeProvider } from "@mui/material/styles";

const theme = createTheme({
  palette: {
    type: "light",
    primary: {
      main: "#ffa726",
    },
    secondary: {
      main: "#147aff",
    },
    success: {
      main: "#4CAF50",
    },
    notConnected: {
      main: "#757575",
    },
    loading: {
      main: "#E64A19",
    },
    selector: {
      main: "#329D9C",
    },
  },
  typography: {
    fontFamily: "Roboto",
  },
});

interface RoboticsThemeProps {
  children: ReactNode;
}

function RoboticsTheme({ children }: RoboticsThemeProps) {
  return <ThemeProvider theme={theme}>{children}</ThemeProvider>;
}

export default RoboticsTheme;
