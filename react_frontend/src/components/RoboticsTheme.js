import {createTheme, ThemeProvider} from '@mui/material/styles';

const theme  = createTheme({
  palette: {
    type: 'light',
    primary: {
      main: '#ffac15',
    },
    secondary: {
      main: '#147aff'
    },
  },
});

function RoboticsTheme(props) {
  return(
      <ThemeProvider theme={theme}>
        {props.children}
      </ThemeProvider>
  )
}

export default RoboticsTheme;