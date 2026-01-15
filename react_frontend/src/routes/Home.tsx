import React from "react";
import { AcademyHeader } from "Components/headers";
import AcademyContainer from "Components/layouts/AcademyContainer";
import { HomepageProvider } from "Contexts/HomepageContext";

const App = () => {
  return (
    <HomepageProvider>
      <AcademyHeader />
      <AcademyContainer />
    </HomepageProvider>
  );
};

export default App;
