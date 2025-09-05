import "../styles/App.css";
import { HomepageProvider } from "../contexts/HomepageContext";
import { AcademyHeader } from "./headers";
import { ErrorProvider, Theme, ThemeProvider } from "jderobot-ide-interface";
import AcademyContainer from "./layouts/AcademyContainer";

const darkTheme: Theme = {
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

const App = () => {
  return (
    <HomepageProvider>
      <ErrorProvider>
        <ThemeProvider theme={darkTheme}>
          <AcademyHeader />
          <AcademyContainer />
        </ThemeProvider>
      </ErrorProvider>
    </HomepageProvider>
  );
};

export default App;
