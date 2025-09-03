import { StyledHeaderButton } from "Components/HeaderMenu/HeaderMenu.styles";
import { useTheme } from "jderobot-ide-interface";
import SchoolRoundedIcon from "@mui/icons-material/SchoolRounded";

const TheoryButton = ({ url }: { url?: string }) => {
  const theme = useTheme();

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
      id="reset-app"
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
