import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import { LogoIcon } from "Icons/index";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import {
  StyledHeaderButtonContainer,
  StyledHeaderText,
} from "./HeaderMenu.styles";

import { ForumButton, SearchButton } from "Components/buttons";

const AcademyHeader = () => {
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
        <a href="https://jderobot.github.io/">
          <LogoIcon
            style={{ width: "40px", height: "40px", marginRight: "10px" }}
          />
        </a>
        <StyledHeaderText color={theme.palette.text}>
          Robotics Academy by JdeRobot
        </StyledHeaderText>

        <StyledHeaderButtonContainer>
          <SearchButton />
          <ForumButton />
        </StyledHeaderButtonContainer>
      </Toolbar>
    </AppBar>
  );
};

export default AcademyHeader;
