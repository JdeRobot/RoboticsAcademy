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
} from "./HeaderMenu.styles";

import { subscribe, unsubscribe } from "Helpers/utils";

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
} from "Components/buttons";

const ExerciseHeader = ({
  project,
  url,
  manager,
  setLayout,
  hasDLModel,
}: {
  project: string;
  url?: string;
  manager: CommsManager | null;
  setLayout: Function;
  hasDLModel: boolean;
}) => {
  const theme = useAcademyTheme();
  const [appRunning, setAppRunning] = useState(false);
  const [dlModel, setDLModel] = useState<string>("");

  const updateState = (e: any) => {
    setAppRunning(e.detail.state === states.RUNNING);
  };

  useEffect(() => {
    subscribe("CommsManagerStateChange", updateState);

    return () => {
      unsubscribe("CommsManagerStateChange", () => {});
    };
  }, []);

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
          {hasDLModel && <DeepLearningButton setModel={setDLModel} />}
          <UploadButton />
          <DownloadButton />
          <LayoutButton setLayout={setLayout} />
          <PlayPauseButton
            project={project}
            manager={manager}
            appRunning={appRunning}
            setAppRunning={setAppRunning}
            dlModel={dlModel}
            hasDLModel={hasDLModel}
          />
          <ResetButton manager={manager} setAppRunning={setAppRunning} />
          <TerminateUniverseButton
            manager={manager}
            setAppRunning={setAppRunning}
          />
          <TheoryButton url={url} />
          <ForumButton />
        </StyledHeaderButtonContainer>
      </Toolbar>
    </AppBar>
  );
};

export default ExerciseHeader;
