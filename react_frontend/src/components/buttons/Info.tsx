import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import InfoOutlinedIcon from "@mui/icons-material/InfoOutlined";
import React from "react";

const USER_GUIDE_URL = "https://jderobot.github.io/RoboticsAcademy/user_guide/";

const InfoButton = () => {
  const theme = useAcademyTheme();

  const openInNewTab = (url: string) => {
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
      onClick={() => openInNewTab(USER_GUIDE_URL)}
      title="Open User Guide"
    >
      <InfoOutlinedIcon htmlColor={theme.palette.text} />
    </StyledHeaderButton>
  );
};

export default InfoButton;
