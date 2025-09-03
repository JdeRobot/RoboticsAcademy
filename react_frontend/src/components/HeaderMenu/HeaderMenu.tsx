import { useContext, useEffect, useRef, useState } from "react";
import JSZip from "jszip";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import { CommsManager, states } from "jderobot-commsmanager";
import commons from "../../common.zip";
import "./HeaderMenu.css";
import {
  LayoutIcon,
  LogoIcon,
  RunIcon,
  StopIcon,
  ResetIcon,
} from "Icons/index";
import { useError, useTheme } from "jderobot-ide-interface";
import { getProjectExtraFiles } from "../../helpers/api";
import { useExercise } from "Contexts/ExerciseContext";
import { StyledHeaderText, StyledProject } from "./HeaderMenu.styles";

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
}: {
  project: string;
  url?: string;
  manager: CommsManager | null;
  setLayout: Function;
}) => {
  const { warning, error } = useError();
  const exerciseContext = useExercise();
  const [isCodeUpdated, _updateCode] = useState<boolean | undefined>(false);
  const [appRunning, setAppRunning] = useState(false);
  const codeRef = useRef("");
  const theme = useTheme();

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
    console.log("Update2222222222222", exerciseContext);
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

  const openInNewTab = (url: URL) => {
    const newWindow = window.open(url, "_blank");
    if (newWindow) {
      newWindow.focus();
    } else {
      console.error("Failed to open new tab/window.");
    }
  };

  // Modal handling

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
          <LogoIcon className="bt-jde-icon" />
        </a>
        <StyledHeaderText color={theme.palette.text}>
          Robotics Academy by JdeRobot
        </StyledHeaderText>

        <StyledProject color={theme.palette.text}>
          <div>{project}</div>
        </StyledProject>
        <div className="bt-header-button-container">
          {/* <Dropdown
            className="bt-header-button"
            id="open-settings-manager"
            title="Layout"
            width={120}
            down
            setter={setLayout}
            possibleValues={["only-editor", "only-viewers", "both"]}
          >
            <LayoutIcon
              className="bt-header-icon"
              stroke={theme.palette.text}
            />
          </Dropdown> */}
          <button
            className="bt-header-button"
            id="run-app"
            onClick={() => onAppStateChange(undefined)}
            title="Run app"
          >
            {appRunning ? (
              <StopIcon className="bt-header-icon" fill={theme.palette.text} />
            ) : (
              <RunIcon className="bt-header-icon" fill={theme.palette.text} />
            )}
          </button>
          <button
            className="bt-header-button"
            id="reset-app"
            onClick={onResetApp}
            title="Reset app"
          >
            <ResetIcon className="bt-header-icon" stroke={theme.palette.text} />
          </button>
          {url && (
            <button
              className="bt-header-button"
              id="reset-app"
              onClick={() => {
                openInNewTab(new URL(url));
              }}
              title="Go to exercise page"
            >
              <ResetIcon
                className="bt-header-icon"
                stroke={theme.palette.text}
              />
            </button>
          )}
          <button
            className="bt-header-button"
            id="reset-app"
            onClick={() => {
              openInNewTab(new URL("https://forum.unibotics.org/"));
            }}
            title="Go to forum"
          >
            <ResetIcon className="bt-header-icon" stroke={theme.palette.text} />
          </button>
        </div>
      </Toolbar>
    </AppBar>
  );
};

export default HeaderMenu;

const Dropdown = ({
  className,
  id,
  title,
  width,
  down,
  setter,
  possibleValues,
  children,
}: {
  className: string;
  id: string;
  title: string;
  width: number;
  down: boolean;
  setter: Function;
  possibleValues: any[];
  children: any;
}) => {
  const [open, setOpen] = useState<boolean>(false);
  const [right, setRight] = useState<any>(width / 2 + 13);
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
      <button
        className={className}
        id={id}
        title={title}
        onClick={(e) => {
          checkPosition(e.clientX);
          e.preventDefault();
          setOpen(!open);
        }}
      >
        {children}
      </button>
      {open && (
        <div
          className="bt-dropdown-list"
          style={{ width: `${width}px`, left: `${right}px` }}
        >
          {possibleValues.map((name, index) => (
            <button onClick={(e: any) => changeValue(e, name)}>{name}</button>
          ))}
        </div>
      )}
    </div>
  );
};
