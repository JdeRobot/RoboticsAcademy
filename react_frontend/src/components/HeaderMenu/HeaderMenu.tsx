import { useEffect, useRef, useState } from "react";
import JSZip from "jszip";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import { CommsManager, states } from "jderobot-commsmanager";
import commons from "../../common.zip";
import { LogoIcon } from "Icons/index";
import { useError, useTheme } from "jderobot-ide-interface";
import { getProjectExtraFiles } from "../../helpers/api";
import { useExercise } from "Contexts/ExerciseContext";
import {
  StyledDropdown,
  StyledHeaderButton,
  StyledHeaderButtonContainer,
  StyledHeaderText,
  StyledProject,
} from "./HeaderMenu.styles";

import { saveCode } from "Helpers/utils";

import PlayArrowRoundedIcon from "@mui/icons-material/PlayArrowRounded";
import PauseRoundedIcon from "@mui/icons-material/PauseRounded";
import ReplayRoundedIcon from "@mui/icons-material/ReplayRounded";
import SpaceDashboardRoundedIcon from "@mui/icons-material/SpaceDashboardRounded";

import {
  HomeButton,
  UploadButton,
  DownloadButton,
  ForumButton,
  TheoryButton,
  DeepLearningButton,
} from "Components/buttons";

export function subscribe(eventName: string, listener: (e: any) => void) {
  document.addEventListener(eventName, listener);
}

export function unsubscribe(eventName: string, listener: () => void) {
  document.removeEventListener(eventName, listener);
}

export function publish(eventName: string, extra: any = undefined) {
  const event = new CustomEvent(eventName, { detail: extra });
  document.dispatchEvent(event);
}

const HeaderMenu = ({
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
  const { warning, error } = useError();
  const exerciseContext = useExercise();
  const theme = useTheme();
  const [isCodeUpdated, _updateCode] = useState<boolean | undefined>(false);
  const [appRunning, setAppRunning] = useState(false);
  const [dlModel, setDLModel] = useState<string>("");
  const codeRef = useRef("");

  const isCodeUpdatedRef = useRef<boolean | undefined>(undefined);

  const updateCode = (data?: boolean) => {
    isCodeUpdatedRef.current = data;
    _updateCode(data);
  };

  const updateState = (e: any) => {
    setAppRunning(e.detail.state === states.RUNNING);
  };

  useEffect(() => {
    subscribe("CommsManagerStateChange", updateState);
    subscribe("autoSaveCompleted", () => {
      updateCode(true);
    });

    return () => {
      unsubscribe("CommsManagerStateChange", () => {});
      unsubscribe("autoSaveCompleted", () => {});
    };
  }, []);

  useEffect(() => {
    codeRef.current = exerciseContext.code;
  }, [exerciseContext]);

  // RB helpers

  const terminateUniverse = async () => {
    if (!manager) {
      warning(
        "Failed to connect with the Robotics Backend docker. Please make sure it is connected."
      );
      return;
    }
    // Down the RB ladder
    await manager.terminateApplication();
    await manager.terminateTools();
    await manager.terminateUniverse();
  };

  // App handling

  const onAppStateChange = async (save?: boolean) => {
    if (!manager) {
      console.error("Manager is not running");
      warning(
        "Failed to connect with the Robotics Backend docker. Please make sure it is connected."
      );
      return;
    }

    if (
      manager.getState() !== "tools_ready" &&
      manager.getState() !== "application_running" &&
      manager.getState() !== "paused"
    ) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure an universe is selected."
      );
      return;
    }

    if (save === undefined) {
      publish("autoSave");
      updateCode(false);
    }

    if (!isCodeUpdatedRef.current) {
      console.log("Try autosave", isCodeUpdated);
      return setTimeout(onAppStateChange, 100, true);
    }

    if (!appRunning) {
      try {
        const zip = new JSZip();
        const commonsZip = await zip.loadAsync(commons);

        const extraFiles: { name: string; content: string }[] =
          await getProjectExtraFiles(project);

        extraFiles.forEach((file) => {
          commonsZip.file(file.name, file.content);
        });

        commonsZip.file("academy.py", codeRef.current);

        // add onnx file to the zip if it exists
        if (hasDLModel) {
          if (dlModel) {
            commonsZip.file("model.onnx", dlModel);
          } else {
            throw new Error("No ONNX model found.");
          }
        }

        // Convert the blob to base64 using FileReader
        const reader = new FileReader();
        reader.onloadend = async () => {
          const base64data = reader.result; // Get the zip in base64
          // Send the base64 encoded blob
          if (base64data) {
            await manager.run(
              "/workspace/code/academy.py",
              ["academy.py"],
              base64data as string
            );
            console.log("Dockerized app started successfully");
          }
        };

        zip.generateAsync({ type: "blob" }).then(function (content) {
          reader.readAsDataURL(content);
        });

        setAppRunning(true);
        console.log("App started successfully");
      } catch (e: unknown) {
        if (e instanceof Error) {
          console.error("Error running app: " + e.message);
          error("Error running app: " + e.message);
        }
      }
    } else {
      try {
        await manager.pause();
        setAppRunning(false);
        console.log("App paused correctly!");
      } catch (e: unknown) {
        if (e instanceof Error) {
          console.error("Error pausing app: " + e.message);
          error("Error pausing app: " + e.message);
        }
      }
    }
  };

  const onResetApp = async () => {
    if (!manager) {
      console.error("Manager is not running");
      warning(
        "Failed to connect with the Robotics Backend docker. Please make sure it is connected."
      );
      return;
    }

    if (
      manager.getState() !== "tools_ready" &&
      manager.getState() !== "application_running" &&
      manager.getState() !== "paused"
    ) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure an universe is selected."
      );
      return;
    }

    await manager.terminateApplication();
    console.log("App reseted!");
    setAppRunning(false);
  };

  const loadFile = (event: React.ChangeEvent<HTMLInputElement>) => {
    event.preventDefault();
    const fr = new FileReader();
    fr.onload = () => {
      if (fr.result) {
        publish("uploadOnlyCode", { code: (fr.result as string)});
      }
    };
    fr.readAsText(event.target.files?.[0]!);
  };

  const saveFile = (save?: boolean) => {
    if (save === undefined) {
      publish("autoSave");
      updateCode(false);
    }

    if (!isCodeUpdatedRef.current) {
      console.log("Try autosave", isCodeUpdated);
      return setTimeout(saveFile, 100, true);
    }

    saveCode("academy", codeRef.current);
  };

  // TODO: project -> center in the middle

  return (
    <AppBar position="static">
      <Toolbar
        style={{
          backgroundColor: theme.palette.primary,
          height: "40px",
          minHeight: "40px",
        }}
      >
        <a href="http://127.0.0.1:7164/exercises/">
          <LogoIcon
            style={{ width: "32px", height: "32px", marginRight: "10px" }}
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
          <UploadButton loadFile={loadFile} />
          <DownloadButton saveFile={() => saveFile(undefined)} />
          <Dropdown
            id="open-settings-manager"
            title="Layout"
            width={120}
            setter={setLayout}
            possibleValues={["only-editor", "only-viewers", "both"]}
          >
            <SpaceDashboardRoundedIcon htmlColor={theme.palette.text} />
          </Dropdown>
          <StyledHeaderButton
            bgColor={theme.palette.primary}
            hoverColor={theme.palette.secondary}
            roundness={theme.roundness}
            id="run-app"
            onClick={() => onAppStateChange(undefined)}
            title="Run app"
          >
            {appRunning ? (
              <PauseRoundedIcon htmlColor={theme.palette.text} />
            ) : (
              <PlayArrowRoundedIcon htmlColor={theme.palette.text} />
            )}
          </StyledHeaderButton>
          <StyledHeaderButton
            bgColor={theme.palette.primary}
            hoverColor={theme.palette.secondary}
            roundness={theme.roundness}
            id="reset-app"
            onClick={onResetApp}
            title="Reset app"
          >
            <ReplayRoundedIcon htmlColor={theme.palette.text} />
          </StyledHeaderButton>
          <TheoryButton url={url} />
          <ForumButton />
        </StyledHeaderButtonContainer>
      </Toolbar>
    </AppBar>
  );
};

export default HeaderMenu;

const Dropdown = ({
  id,
  title,
  width,
  setter,
  possibleValues,
  children,
}: {
  id: string;
  title: string;
  width: number;
  setter: Function;
  possibleValues: any[];
  children: any;
}) => {
  const [open, setOpen] = useState<boolean>(false);
  const [right, setRight] = useState<any>(width / 2 + 13);
  const theme = useTheme();
  const dropdown = useRef<HTMLDivElement>(null);

  const changeValue = (e: any, value: any) => {
    e.preventDefault();
    setter(value);
    setOpen(false);
  };

  const closeOpenMenus = (e: any) => {
    if (open && !dropdown.current?.contains(e.target)) {
      setOpen(false);
    }
  };

  const checkPosition = (x: number) => {
    if (x + width / 2 > window.innerWidth) {
      // To the left
      setRight(x);
    } else if (x < width / 2) {
      // To the right
      setRight(x - width);
    } else {
      // In the middle
      setRight(x - width / 2 + 13);
    }
  };

  document.addEventListener("mousedown", closeOpenMenus);

  return (
    <div ref={dropdown}>
      <StyledHeaderButton
        bgColor={theme.palette.primary}
        hoverColor={theme.palette.secondary}
        roundness={theme.roundness}
        id={id}
        title={title}
        onClick={(e) => {
          checkPosition(e.clientX);
          e.preventDefault();
          setOpen(!open);
        }}
      >
        {children}
      </StyledHeaderButton>
      {open && (
        <StyledDropdown
          color={theme.palette.text}
          bgColor={theme.palette.primary}
          hoverColor={theme.palette.secondary}
          roundness={theme.roundness}
          style={{ width: `${width}px`, left: `${right}px` }}
        >
          {possibleValues.map((name, index) => (
            <button onClick={(e: any) => changeValue(e, name)}>{name}</button>
          ))}
        </StyledDropdown>
      )}
    </div>
  );
};
