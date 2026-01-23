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
import React from "react";

import PlayArrowRoundedIcon from "@mui/icons-material/PlayArrowRounded";
import PauseRoundedIcon from "@mui/icons-material/PauseRounded";
import SyncRoundedIcon from "@mui/icons-material/SyncRounded";
import { getFileList, getHelperFileList } from "Api";

const PlayPauseButton = ({
  project,
  language,
  connectManager,
}: {
  project: string;
  language?: string;
  connectManager: (
    desiredState?: string,
    callback?: () => void
  ) => Promise<void>;
}) => {
  const theme = useAcademyTheme();
  const { warning, error, info, close } = useError();
  const filesRef = useRef<Entry[]>([]);
  const runningFilesRef = useRef<Entry[]>([]);
  const runningEntrypointRef = useRef<string | undefined>(undefined);
  const [state, setState] = useState<string>(
    CommsManager.getInstance().getState()
  );
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

  useEffect(() => {
    subscribe("autoSaveCompleted", () => {
      updateCode(true);
    });
    subscribe("CommsManagerStateChange", updateState);

    return () => {
      unsubscribe("autoSaveCompleted", () => {});
      unsubscribe("CommsManagerStateChange", () => {});
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

    if (save === undefined) {
      publish("autoSave");
      updateCode(false);
    }

    if (!isCodeUpdatedRef.current) {
      return setTimeout(onAppStateChange, 100, true);
    }

    const files = await getFileList(project);
    filesRef.current = JSON.parse(files)

    if (state === states.PAUSED) {
      if (
        runningFilesRef.current === filesRef.current &&
        runningEntrypointRef.current === language
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
      const extension = language === "cpp" ? "cpp" : "py";
      let commonsZip;
      let toLint = [""];

      if (extension === "py") {
        commonsZip = await zip.loadAsync(commons);
        toLint = ["academy.py"];
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
        language ?? "python"
      );

      await zipCodeFiles(commonsZip, filesRef.current, project);

      runningFilesRef.current = filesRef.current;
      runningEntrypointRef.current = language;

      // Convert the blob to base64 using FileReader
      const reader = new FileReader();
      reader.onloadend = async () => {
        const base64data = reader.result; // Get the zip in base64
        // Send the base64 encoded blob
        if (base64data) {
          try {
            await manager.run(
              `/workspace/code/academy.${extension}`,
              toLint,
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
