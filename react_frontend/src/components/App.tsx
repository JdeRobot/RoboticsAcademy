import { HomepageProvider } from "../contexts/HomepageContext";
import { AcademyHeader } from "./headers";
import { ErrorProvider } from "jderobot-ide-interface";
import AcademyContainer from "./layouts/AcademyContainer";
import { AcademyThemeProvider } from "Contexts/AcademyThemeContext";

const App = () => {
  return (
    <HomepageProvider>
      <ErrorProvider>
        <AcademyThemeProvider>
          <AcademyHeader />
          <AcademyContainer />
        </AcademyThemeProvider>
      </ErrorProvider>
    </HomepageProvider>
  );
};

export default App;
