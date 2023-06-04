import { createTheme, ThemeProvider } from "@mui/material/styles";
import PropTypes from "prop-types";

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

function RoboticsTheme(props) {
  return <ThemeProvider theme={theme}>{props.children}</ThemeProvider>;
}

RoboticsTheme.propTypes = {
  children: PropTypes.node,
};

export default RoboticsTheme;
