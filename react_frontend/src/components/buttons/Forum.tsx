import { StyledHeaderButton } from "Components/headers/HeaderMenu.styles";
import { useTheme } from "jderobot-ide-interface";
import ForumRoundedIcon from "@mui/icons-material/ForumRounded";

const ForumButton = () => {
  const theme = useTheme();

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
      id="reset-app"
      onClick={() => {
        openInNewTab(new URL("https://forum.unibotics.org/"));
      }}
      title="Go to forum"
    >
      <ForumRoundedIcon htmlColor={theme.palette.text} />
    </StyledHeaderButton>
  );
};

export default ForumButton;
