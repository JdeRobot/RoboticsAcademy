import {createTheme, ThemeProvider} from '@mui/material/styles';

const theme  = createTheme({
  palette: {
    type: 'light',
    primary: {
      main: '#ffa726',
    },
    secondary: {
      main: '#147aff'
    },
    success:{
      main: '#4CAF50'
    },
    notConnected: {
      main: '#757575'
    },
    loading: {
      main: '#E64A19'
    }
  },
  typography:{
    fontFamily: 'Roboto'
  }
});

function RoboticsTheme(props) {
  return(
      <ThemeProvider theme={theme}>
        {props.children}
      </ThemeProvider>
  )
}

export default RoboticsTheme;