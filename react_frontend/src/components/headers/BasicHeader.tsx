import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import { LogoIcon } from "Icons/index";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { StyledHeaderButtonContainer } from "Styles/headers/HeaderMenu.styles";

import { ForumButton } from "Components/buttons";
import { HomeButton, ThemeButton } from "Components/buttons";
import { Link } from "react-router-dom";

const BasicHeader = () => {
  const theme = useAcademyTheme();
  return (
    <AppBar position="static">
      <Toolbar
        style={{
          backgroundColor: theme.palette.primary,
          height: "50px",
          minHeight: "50px",
        }}
      >
        <Link to=".." viewTransition>
          <LogoIcon
            style={{ width: "40px", height: "40px", marginRight: "10px" }}
          />
        </Link>

        <StyledHeaderButtonContainer>
          <HomeButton />
          <ThemeButton />
          <ForumButton />
        </StyledHeaderButtonContainer>
      </Toolbar>
    </AppBar>
  );
};

export default BasicHeader;
