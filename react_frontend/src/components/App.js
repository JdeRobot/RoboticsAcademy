import '../styles/App.css';
import {ExerciseList} from "./ExerciseList";
import DrawerAppBar from "./DrawerAppBar";
import { createTheme, ThemeProvider } from '@mui/material/styles';

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

function App() {
  return (
      <ThemeProvider theme ={theme}>
          <DrawerAppBar/>
          <ExerciseList />
      </ThemeProvider>
  );
}

export default App;