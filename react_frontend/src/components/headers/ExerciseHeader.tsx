import React from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import { Link } from "react-router-dom";
import { LogoIcon } from "Icons/index";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import {
  StyledHeaderButtonContainer,
  StyledHeaderText,
  StyledProject,
} from "../../styles/headers/HeaderMenu.styles";

import {
  HomeButton,
  DownloadButton,
  ForumButton,
  InfoButton,
  TheoryButton,
  PlayPauseButton,
  ResetButton,
  LayoutButton,
  TerminateUniverseButton,
  ThemeButton,
} from "Components/buttons";
import { Layout } from "jderobot-ide-interface";

const ExerciseHeader = ({
  project,
  name,
  supportedLanguages,
  url,
  setLayout,
  connectManager,
}: {
  project: string;
  name: string;
  supportedLanguages: string[];
  url?: string;
  setLayout: (layout: Layout) => void;
  connectManager: (
    desiredState?: string,
    callback?: () => void
  ) => Promise<void>;
}) => {
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
        <StyledHeaderText color={theme.palette.text}>
          Robotics Academy by JdeRobot
        </StyledHeaderText>

        <StyledProject color={theme.palette.text}>
          <div>{name}</div>
        </StyledProject>
        <StyledHeaderButtonContainer>
          <HomeButton />
          <ThemeButton />
          <DownloadButton project={project} />
          <LayoutButton setLayout={setLayout} />
          <PlayPauseButton
            project={project}
            supportedLanguages={supportedLanguages}
            connectManager={connectManager}
          />
          <ResetButton />
          <TerminateUniverseButton />
          <TheoryButton url={url} />
          <InfoButton />
          <ForumButton />
        </StyledHeaderButtonContainer>
      </Toolbar>
    </AppBar>
  );
};

export default ExerciseHeader;
