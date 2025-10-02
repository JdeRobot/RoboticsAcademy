import { HomepageProvider } from "../contexts/HomepageContext";
import { AcademyHeader } from "./headers";
import { ErrorProvider } from "jderobot-ide-interface";
import AcademyContainer from "./layouts/AcademyContainer";
import { AcademyThemeProvider } from "Contexts/AcademyThemeContext";
import React from "react";

const App = () => {
  return (
    <AcademyThemeProvider>
      <HomepageProvider>
        <ErrorProvider>
          <AcademyHeader />
          <AcademyContainer />
        </ErrorProvider>
      </HomepageProvider>
    </AcademyThemeProvider>
  );
};

export default App;
