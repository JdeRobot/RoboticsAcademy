import React, { useState } from "react";
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
  UploadButton,
  DownloadButton,
  ForumButton,
  InfoButton,
  TheoryButton,
  DeepLearningButton,
  PlayPauseButton,
  ResetButton,
  LayoutButton,
  TerminateUniverseButton,
  ThemeButton,
  LanguageButton,
} from "Components/buttons";
import { Layout } from "jderobot-ide-interface";

const ExerciseHeader = ({
  project,
  language,
  url,
  setLayout,
  setLanguage,
  hasDLModel,
  connectManager,
  saving,
}: {
  project: string;
  language?: string;
  url?: string;
  setLayout: (layout: Layout) => void;
  setLanguage: (language: string) => void;
  hasDLModel: boolean;
  connectManager: (
    desiredState?: string,
    callback?: () => void
  ) => Promise<void>;
  saving?: boolean;
}) => {
  const theme = useAcademyTheme();
  const [dlModel, setDLModel] = useState<ArrayBuffer | undefined>();

  // TODO: project -> center in the middle

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
          <div>{project}</div>
        </StyledProject>
        <StyledHeaderButtonContainer>
          <HomeButton />
          <ThemeButton />
          {language && (
            <LanguageButton
              language={language}
              setter={setLanguage}
              loading={saving}
            />
          )}
          {hasDLModel && <DeepLearningButton setModel={setDLModel} />}
          <UploadButton language={language} />
          <DownloadButton language={language} />
          <LayoutButton setLayout={setLayout} />
          <PlayPauseButton
            project={project}
            language={language}
            dlModel={dlModel}
            hasDLModel={hasDLModel}
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
