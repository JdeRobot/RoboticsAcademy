import React from "react";
import { ErrorProvider } from "jderobot-ide-interface";
import { AcademyThemeProvider } from "Contexts/AcademyThemeContext";
import { Outlet } from "react-router";

const App = () => {
  return (
    <AcademyThemeProvider>
      <ErrorProvider>
        
        <Outlet />
      </ErrorProvider>
    </AcademyThemeProvider>
  );
};

export default App;
