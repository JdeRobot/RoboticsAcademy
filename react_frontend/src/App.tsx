import React from "react";
import { BrowserRouter, Routes, Route } from "react-router-dom";

import { ErrorProvider } from "jderobot-ide-interface";
import { Studio, Home } from "Routes";
import { AcademyThemeProvider } from "Contexts/AcademyThemeContext";

const App = () => {
  return (
    <AcademyThemeProvider>
      <ErrorProvider>
        <BrowserRouter>
          <Routes>
            <Route path="/academy">
              <Route index element={<Home />} />
              <Route path="studio/:proj_id" element={<Studio />} />
            </Route>
          </Routes>
        </BrowserRouter>
      </ErrorProvider>
    </AcademyThemeProvider>
  );
};

export default App;
