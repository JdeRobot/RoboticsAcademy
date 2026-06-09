import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { StyledWebGUIError } from "Styles/visualizers/WebGUIPreview.styles";
import WebGUIContainer from "Components/exercise/WebGUIContainer";
import React from "react";

// camera
const WebGUILoading = () => {
  const theme = useAcademyTheme();

  return (
    <WebGUIContainer>
      <StyledWebGUIError color={theme.palette.primary}>
        <h3>Loading ...</h3>
      </StyledWebGUIError>
    </WebGUIContainer>
  );
};

export default WebGUILoading;
