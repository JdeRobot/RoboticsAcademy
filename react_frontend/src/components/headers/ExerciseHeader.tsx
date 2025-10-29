import { useEffect, useState } from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import { CommsManager, states } from "jderobot-commsmanager";
import { LogoIcon } from "Icons/index";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import {
  StyledHeaderButtonContainer,
  StyledHeaderText,
  StyledProject,
} from "../../styles/headers/HeaderMenu.styles";

import { subscribe, unsubscribe } from "Helpers/utils";
import React from "react";

import {
  HomeButton,
  UploadButton,
  DownloadButton,
  ForumButton,
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
        <a href="http://127.0.0.1:7164/exercises/">
          <LogoIcon
            style={{ width: "40px", height: "40px", marginRight: "10px" }}
          />
        </a>
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
          <ForumButton />
        </StyledHeaderButtonContainer>
      </Toolbar>
    </AppBar>
  );
};

export default ExerciseHeader;
