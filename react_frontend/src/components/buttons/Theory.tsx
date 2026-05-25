import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import SchoolRoundedIcon from "@mui/icons-material/SchoolRounded";
import React from "react";

const TheoryButton = ({ url }: { url?: string }) => {
  const theme = useAcademyTheme();

  const openInNewTab = (url: URL) => {
    const newWindow = window.open(url, "_blank");
    if (newWindow) {
      newWindow.focus();
    } else {
      console.error("Failed to open new tab/window.");
    }
  };

  if (url === undefined) {
    return <></>;
  }

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="theory-button"
      onClick={() => {
        openInNewTab(new URL(url));
      }}
      title="Go to exercise page"
    >
      <SchoolRoundedIcon htmlColor={theme.palette.text} />
    </StyledHeaderButton>
  );
};

export default TheoryButton;
