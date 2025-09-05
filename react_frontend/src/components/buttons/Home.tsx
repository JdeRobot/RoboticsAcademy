import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import HomeRoundedIcon from "@mui/icons-material/HomeRounded";


const HomeButton = () => {
  const theme = useAcademyTheme();

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="return-academy"
      title="Return to Academy"
    >
      <a href="http://127.0.0.1:7164/exercises/">
        <HomeRoundedIcon htmlColor={theme.palette.text} />
      </a>
    </StyledHeaderButton>
  );
};

export default HomeButton;