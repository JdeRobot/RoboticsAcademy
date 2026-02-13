import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
import { Entry, useError } from "jderobot-ide-interface";
import {
  publish,
  subscribe,
  unsubscribe,
  zipCodeFiles,
  zipHelperFiles,
} from "Helpers/utils";
import { CommsManager, states } from "jderobot-commsmanager";
import JSZip from "jszip";
import { useEffect, useRef, useState } from "react";
import commons from "../../common.zip";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";

import PlayArrowRoundedIcon from "@mui/icons-material/PlayArrowRounded";
import PauseRoundedIcon from "@mui/icons-material/PauseRounded";
import SyncRoundedIcon from "@mui/icons-material/SyncRounded";
import { getFileList, getHelperFileList } from "Api";

const PlayPauseButton = ({
  project,
  supportedLanguages,
  connectManager,
}: {
  project: string;
  supportedLanguages: string[];
  connectManager: (
    desiredState?: string,
    callback?: () => void
  ) => Promise<void>;
}) => {
  const theme = useAcademyTheme();
  const { warning, error, info, close } = useError();
  const filesRef = useRef<Entry[]>([]);
  const entrypointRef = useRef<Entry | undefined>(undefined);
  const runningFilesRef = useRef<Entry[]>([]);
  const runningEntrypointRef = useRef<Entry | undefined>(undefined);
  const [state, setState] = useState<string>(states.IDLE);
  const [loading, setLoading] = useState<boolean>(false);
  const isCodeUpdatedRef = useRef<boolean | undefined>(undefined);
  const [, _updateCode] = useState<boolean | undefined>(false);

  const updateCode = (data?: boolean) => {
    isCodeUpdatedRef.current = data;
    _updateCode(data);
  };

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
    subscribe("autoSaveCompleted", () => {
      updateCode(true);
    });
    subscribe("CommsManagerStateChange", updateState);
    subscribe("currentFile", updateCurrent);

    return () => {
      unsubscribe("autoSaveCompleted", () => {});
      unsubscribe("CommsManagerStateChange", () => {});
      unsubscribe("currentFile", () => {});
    };
  }, []);

  useEffect(() => {
    if (
      state === states.RUNNING ||
      state === states.PAUSED ||
      state === states.TOOLS_READY
    ) {
      setLoading(false);
    }
  }, [state]);

  const getLanguage = (extension?: string) => {
    const fileTypes = {
      py: "python",
      cpp: "cpp",
    };

    if (extension === undefined) {
      return undefined;
    }

    for (const key in fileTypes) {
      if (key === extension) {
        return fileTypes[key as keyof typeof fileTypes];
      }
    }

    return undefined;
  };

  // App handling

  const onAppStateChange = async (save?: boolean) => {
    const manager = CommsManager.getInstance();
    const state = manager.getState();

    setLoading(true);

    if (state === states.IDLE) {
      info("Connecting with the Robotics Backend ...");
      connectManager(states.TOOLS_READY, () => {
        setLoading(false);
        close();
        onAppStateChange();
      });
      return;
    }

    if (state === states.WORLD_READY || state === states.CONNECTED) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure an universe is selected."
      );
      setLoading(false);
      return;
    }

    if (state === states.RUNNING) {
      try {
        await manager.pause();
        console.log("App paused correctly!");
      } catch (e: unknown) {
        console.error("Error pausing app: " + (e as Error).message);
        error(
          "Failed to stop the application. See the traces in the terminal."
        );
      }
      setLoading(false);
      return;
    }

    if (entrypointRef.current === undefined) {
      error(
        "Failed to run the application. Make sure to select an entrypoint by opening it in the editor."
      );
      setLoading(false);
      return;
    }

    const language = getLanguage(entrypointRef.current.path.split(".").pop());

    if (language === undefined || !supportedLanguages.includes(language)) {
      error(
        `Failed to run the application. Entrypoint ${entrypointRef.current.path} is not supported.`
      );
      setLoading(false);
      return;
    }

    if (save === undefined) {
      publish("autoSave");
      updateCode(false);
    }

    if (!isCodeUpdatedRef.current) {
      return setTimeout(onAppStateChange, 100, true);
    }

    const files = await getFileList(project);
    filesRef.current = JSON.parse(files);

    if (state === states.PAUSED) {
      if (
        runningFilesRef.current === filesRef.current &&
        runningEntrypointRef.current === entrypointRef.current
      ) {
        try {
          await manager.resume();
          console.log("App resumed correctly!");
        } catch (e: unknown) {
          console.error("Error resuming app: " + (e as Error).message);
          error(
            "Failed to resume the application. See the traces in the terminal."
          );
        }
        setLoading(false);
        return;
      }
    }

    try {
      const zip = new JSZip();
      const extension = entrypointRef.current.path.split(".").pop();
      let commonsZip;

      if (extension === "py") {
        commonsZip = await zip.loadAsync(commons);
      } else {
        commonsZip = zip;
      }

      const helper_files = await getHelperFileList(
        project,
        language ?? "python"
      );

      await zipHelperFiles(
        commonsZip,
        helper_files,
        project,
        language ?? "python",
        entrypointRef.current
      );

      await zipCodeFiles(commonsZip, filesRef.current, project);

      runningFilesRef.current = filesRef.current;
      runningEntrypointRef.current = entrypointRef.current;

      // Convert the blob to base64 using FileReader
      const reader = new FileReader();
      reader.onloadend = async () => {
        const base64data = reader.result; // Get the zip in base64
        // Send the base64 encoded blob
        if (base64data && runningEntrypointRef.current) {
          try {
            await manager.run(
              `/workspace/code/${runningEntrypointRef.current.path}`,
              [runningEntrypointRef.current.path],
              base64data as string
            );
          } catch {
            error(
              "Failed to run the application. See the traces in the terminal."
            );
            setLoading(false);
          }
          console.log("Dockerized app started successfully");
        }
      };

      commonsZip.generateAsync({ type: "blob" }).then(function (content: Blob) {
        reader.readAsDataURL(content);
      });

      console.log("App started successfully");
    } catch (e: unknown) {
      setLoading(false);
      if (e instanceof Error) {
        console.error("Error running app: " + e.message);
        error("Error running app: " + e.message);
      }
    }
  };

  return (
    <>
      <StyledHeaderButton
        bgColor={theme.palette.primary}
        hoverColor={theme.palette.secondary}
        roundness={theme.roundness}
        id="run-app"
        onClick={() => onAppStateChange(undefined)}
        title="Run app"
        disabled={loading}
      >
        {loading ? (
          <SyncRoundedIcon htmlColor={theme.palette.text} id="loading-spin" />
        ) : (
          <>
            {state === states.RUNNING ? (
              <PauseRoundedIcon htmlColor={theme.palette.text} />
            ) : (
              <PlayArrowRoundedIcon htmlColor={theme.palette.text} />
            )}
          </>
        )}
      </StyledHeaderButton>
    </>
  );
};

export default PlayPauseButton;
