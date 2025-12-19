import { ThemeProvider } from "jderobot-ide-interface";
import { createContext, ReactNode, useContext, useState } from "react";
import { AcademyTheme } from "Types/index";
import React from "react";

interface AcademyThemeProviderProps {
  theme?: any;
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
    warning: "#b87a09",
    error: "#b91c1cff",
    bgDark: "#16161d",
    bg: "#16161d",
    bgLight: "#16161d",
    primary: "#ffa726",
    secondary: "#ff8800",
    scrollbar: "#6f6f90",
    border: {
      warning: "#af5500",
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
      warning: "#af5500",
      info: "#134f53",
      hoverError: "#c63939",
      hoverSuccess: "#29ac29",
      hoverWarning: "#e05a00",
      hoverInfo: "#1d777c",
    },
    selectedGradient:
      "linear-gradient( -45deg, #12494c 0%, #584f42 50%, #909c7b 100%)",
  },
  roundness: 5,
  viewRoundness: 0,
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
    bgDark: "#ffffff",
    bg: "#ffffff",
    bgLight: "#ffffff",
    primary: "#ffa726",
    secondary: "#ff8800",
    scrollbar: "#6f6f90",
    border: {
      warning: "#af5500",
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
      warning: "#af5500",
      info: "#134f53",
      hoverError: "#c63939",
      hoverSuccess: "#29ac29",
      hoverWarning: "#e05a00",
      hoverInfo: "#1d777c",
    },
    selectedGradient:
      "linear-gradient( -45deg, #12494c 0%, #584f42 50%, #909c7b 100%)",
  },
  roundness: 5,
  viewRoundness: 0,
  monacoTheme: "light",
};

const AcademyThemeContext = createContext(darkTheme);
export const useAcademyTheme = () => useContext(AcademyThemeContext);

export const AcademyThemeProvider = ({
  theme,
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

  if (theme) {
    return (
      <AcademyThemeContext.Provider value={theme}>
        <ThemeProvider theme={currentTheme}>{children}</ThemeProvider>
      </AcademyThemeContext.Provider>
    );
  }

  return (
    <AcademyThemeContext.Provider
      value={{ ...currentTheme, switch: themeSwitchHandler }}
    >
      <ThemeProvider theme={currentTheme}>{children}</ThemeProvider>
    </AcademyThemeContext.Provider>
  );
};
