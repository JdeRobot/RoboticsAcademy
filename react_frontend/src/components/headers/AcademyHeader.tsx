import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import { LogoIcon } from "Icons/index";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import {
  StyledHeaderButtonContainer,
  StyledHeaderText,
} from "../../styles/headers/HeaderMenu.styles";

import {
  ForumButton,
  SearchButton,
  ThemeButton,
  InfoButton,
} from "Components/buttons";
import React from "react";

const AcademyHeader = () => {
  const theme = useAcademyTheme();

  console.log(theme);

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
          <ThemeButton />
          <InfoButton />
          <ForumButton />
        </StyledHeaderButtonContainer>
      </Toolbar>
    </AppBar>
  );
};

export default AcademyHeader;
