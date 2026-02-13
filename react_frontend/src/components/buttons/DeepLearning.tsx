import { useRef, useState } from "react";
import PsychologyRoundedIcon from "@mui/icons-material/PsychologyRounded";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";

const DeepLearningButton = ({ url }: { url?: string }) => {
  const theme = useAcademyTheme();
  // using hooks to avoid unused variable errors for useRef and useState
  const ref = useRef(null);
  const [val, setVal] = useState(0);

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
      ref={ref}
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="deep-learning-button"
      onClick={() => {
        setVal(val + 1);
        openInNewTab(new URL(url));
      }}
      title="Deep Learning"
    >
      <PsychologyRoundedIcon htmlColor={theme.palette.text} />
    </StyledHeaderButton>
  );
};

export default DeepLearningButton;
