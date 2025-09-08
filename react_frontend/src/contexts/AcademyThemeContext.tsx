import { ThemeProvider } from "jderobot-ide-interface";
import { createContext, ReactNode, useContext, useState } from "react";
import { AcademyTheme } from "Types/index";
import React from "react";

interface AcademyThemeProviderProps {
  children?: ReactNode;
}

const darkTheme: AcademyTheme = {
  switch: () => {},
  viewer3d: {
    grid: "#ededf2",
    background: "#16161d",
  },
  palette: {
    darkText: "#ededf2",
    text: "#000000ff",
    placeholderText: "#a6a6bf",
    success: "#29ac29",
    warning: "#af5500ff",
    error: "#802626",
    background: "#16161d",
    primary: "#ffa726",
    secondary: "#ff8800",
    scrollbar: "#6f6f90",
    border: {
      warning: "#af5500ff",
      error: "#772222",
      info: "#134f53",
    },
    progressBar: {
      background: "#134f53",
      color: "#1d777c",
    },
    button: {
      error: "#9e2e2e",
      success: "#29ac29",
      warning: "#af5500ff",
      info: "#134f53",
      hoverError: "#c63939",
      hoverSuccess: "#29ac29",
      hoverWarning: "#e05a00ffff",
      hoverInfo: "#1d777c",
    },
    selectedGradient:
      "linear-gradient( -45deg, #12494c 0%, #584f42 50%, #909c7b 100%)",
  },
  roundness: 5,
  monacoTheme: "dark",
};

const lightTheme: AcademyTheme = {
  switch: () => {},
  viewer3d: {
    grid: "#000000",
    background: "#ffffff",
  },
  palette: {
    darkText: "#ededf2",
    text: "#000000ff",
    placeholderText: "#a6a6bf",
    success: "#29ac29",
    warning: "#af5500ff",
    error: "#802626",
    background: "#ffffff",
    primary: "#ffa726",
    secondary: "#ff8800",
    scrollbar: "#6f6f90",
    border: {
      warning: "#af5500ff",
      error: "#772222",
      info: "#134f53",
    },
    progressBar: {
      background: "#134f53",
      color: "#1d777c",
    },
    button: {
      error: "#9e2e2e",
      success: "#29ac29",
      warning: "#af5500ff",
      info: "#134f53",
      hoverError: "#c63939",
      hoverSuccess: "#29ac29",
      hoverWarning: "#e05a00ffff",
      hoverInfo: "#1d777c",
    },
    selectedGradient:
      "linear-gradient( -45deg, #12494c 0%, #584f42 50%, #909c7b 100%)",
  },
  roundness: 5,
  monacoTheme: "light",
};

const AcademyThemeContext = createContext(darkTheme);
export const useAcademyTheme = () => useContext(AcademyThemeContext);

export const AcademyThemeProvider = ({
  children,
}: AcademyThemeProviderProps) => {
  const [currentTheme, setCurrentTheme] = useState<AcademyTheme>(
    window.localStorage.getItem("themeType") !== null
      ? window.localStorage.getItem("themeType") === "light"
        ? lightTheme
        : darkTheme
      : darkTheme
  );

  const themeSwitchHandler = (themeType: string) => {
    window.localStorage.setItem("themeType", themeType);
    switch (themeType) {
      case "light":
        setCurrentTheme(lightTheme);
        break;
      case "dark":
        setCurrentTheme(darkTheme);
        break;
      default:
        break;
    }
  };

  return (
    <AcademyThemeContext.Provider
      value={{ ...currentTheme, switch: themeSwitchHandler }}
    >
      <ThemeProvider theme={currentTheme}>{children}</ThemeProvider>
    </AcademyThemeContext.Provider>
  );
};

// declare module "@mui/material/styles" {
//   interface Palette {
//     notConnected: Palette["primary"];
//     loading: Palette["primary"];
//     selector: Palette["primary"];
//   }
//   interface PaletteOptions {
//     notConnected?: PaletteOptions["primary"];
//     loading?: PaletteOptions["primary"];
//     selector?: PaletteOptions["primary"];
//   }
// }

// const theme = createTheme({
//   palette: {
//     mode: "light",
//     primary: {
//       main: "#ffa726",
//     },
//     secondary: {
//       main: "#147aff",
//     },
//     success: {
//       main: "#4CAF50",
//     },
//     notConnected: {
//       main: "#757575",
//     },
//     loading: {
//       main: "#E64A19",
//     },
//     selector: {
//       main: "#329D9C",
//     },
//   },
//   typography: {
//     fontFamily: "Roboto",
//   },
// });
