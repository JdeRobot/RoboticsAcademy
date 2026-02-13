import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import ErrorOutlineRoundedIcon from "@mui/icons-material/ErrorOutlineRounded";
import { StyledWebGUIError } from "Styles/visualizers/WebGUIPreview.styles";
import WebGUIContainer from "Components/exercise/WebGUIContainer";

// camera
const WebGUIPreview = () => {
  const theme = useAcademyTheme();

  return (
    <WebGUIContainer>
      <StyledWebGUIError color={theme.palette.error}>
        <ErrorOutlineRoundedIcon htmlColor={theme.palette.error} />
        <h3>Error: WebGUI.js failed to load</h3>
      </StyledWebGUIError>
    </WebGUIContainer>
  );
};

export default WebGUIPreview;
