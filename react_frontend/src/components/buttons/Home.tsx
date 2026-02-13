import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import HomeRoundedIcon from "@mui/icons-material/HomeRounded";
import { Link } from "react-router-dom";

const HomeButton = () => {
  const theme = useAcademyTheme();

  return (
    <Link to=".." viewTransition>
      <StyledHeaderButton
        bgColor={theme.palette.primary}
        hoverColor={theme.palette.secondary}
        roundness={theme.roundness}
        id="return-academy"
        title="Return to Academy"
      >
        <HomeRoundedIcon htmlColor={theme.palette.text} />
      </StyledHeaderButton>
    </Link>
  );
};

export default HomeButton;
