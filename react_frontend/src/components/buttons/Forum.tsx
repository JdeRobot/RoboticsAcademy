import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import ForumRoundedIcon from "@mui/icons-material/ForumRounded";
import React from "react";

const ForumButton = () => {
  const theme = useAcademyTheme();

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
      id="forum-button"
      onClick={() => {
        openInNewTab(
          new URL("https://github.com/JdeRobot/RoboticsAcademy/discussions")
        );
      }}
      title="Go to forum"
    >
      <ForumRoundedIcon htmlColor={theme.palette.text} />
    </StyledHeaderButton>
  );
};

export default ForumButton;
