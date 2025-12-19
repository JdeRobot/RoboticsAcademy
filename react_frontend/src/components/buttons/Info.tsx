import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import InfoIcon from "@mui/icons-material/InfoRounded";
import React from "react";

const InfoButton = () => {
  const theme = useAcademyTheme();
  const url = "https://jderobot.github.io/RoboticsAcademy/user_guide/";

  const openInNewTab = (url: URL) => {
    const newWindow = window.open(url, "_blank");
    if (newWindow) {
      newWindow.focus();
    } else {
      console.error("Failed to open new tab/window.");
    }
  };

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="info-button"
      onClick={() => openInNewTab(new URL(url))}
      title="Open User Guide"
    >
      <InfoIcon htmlColor={theme.palette.text} />
    </StyledHeaderButton>
  );
};

export default InfoButton;
