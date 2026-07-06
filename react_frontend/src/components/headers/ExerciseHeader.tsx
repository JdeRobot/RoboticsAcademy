import React, { MutableRefObject, useEffect, useRef, useState } from "react";
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
  ConnectButton,
} from "Components/buttons";
import { Entry, Layout } from "jderobot-ide-interface";
import { CommsManager } from "jderobot-commsmanager";
import { subscribe, unsubscribe } from "Helpers/utils";

const ExerciseHeader = ({
  project,
  name,
  supportedLanguages,
  url,
  setLayout,
  connectManager,
  commsManager,
  userRef,
}: {
  project: string;
  name: string;
  supportedLanguages: string[];
  url?: string;
  setLayout: (layout: Layout) => void;
  commsManager: CommsManager | null;
  userRef: MutableRefObject<string | undefined>;
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
          <ExecutionControl
            project={project}
            supportedLanguages={supportedLanguages}
            commsManager={commsManager}
            connectManager={connectManager}
            userRef={userRef}
          />
          <TheoryButton url={url} />
          <InfoButton />
          <ForumButton />
        </StyledHeaderButtonContainer>
      </Toolbar>
    </AppBar>
  );
};

const ExecutionControl = ({
  project,
  supportedLanguages,
  commsManager,
  connectManager,
  userRef,
}: {
  project: string;
  supportedLanguages: string[];
  commsManager: CommsManager | null;
  userRef: MutableRefObject<string | undefined>;
  connectManager: (
    desiredState?: string,
    callback?: () => void
  ) => Promise<void>;
}) => {
  const [state, setState] = useState<string | undefined>(
    commsManager?.getState()
  );
  const entrypointRef = useRef<Entry | undefined>(undefined);

  const updateState = (e: unknown) => {
    const T = CustomEvent<{ detail: unknown }>;
    if (e instanceof T) {
      setState(e.detail.state);
    }
  };

  const updateCurrent = (e: unknown) => {
    const T = CustomEvent<{ detail: { file?: Entry } }>;
    if (e instanceof T) {
      entrypointRef.current = e.detail.file;
    }
  };

  useEffect(() => {
    subscribe("CommsManagerStateChange", updateState);
    subscribe("currentFile", updateCurrent);

    return () => {
      unsubscribe("CommsManagerStateChange", () => {});
      unsubscribe("currentFile", () => {});
    };
  }, []);

  const viewConnect = state === undefined || state === "idle";

  return (
    <>
      {viewConnect ? (
        <ConnectButton connectManager={connectManager} />
      ) : (
        <>
          <PlayPauseButton
            project={project}
            supportedLanguages={supportedLanguages}
            userRef={userRef}
            entrypointRef={entrypointRef}
          />
          <ResetButton />
          <TerminateUniverseButton />
        </>
      )}
    </>
  );
};

export default ExerciseHeader;
