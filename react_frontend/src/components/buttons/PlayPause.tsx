import React, { RefObject, useEffect, useRef, useState } from "react";
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
import commons from "../../common.zip";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";

import PlayArrowRoundedIcon from "@mui/icons-material/PlayArrowRounded";
import PauseRoundedIcon from "@mui/icons-material/PauseRounded";
import SyncRoundedIcon from "@mui/icons-material/SyncRounded";
import { getFileList, getHelperFileList } from "Api";

const PlayPauseButton = ({
  project,
  supportedLanguages,
  userRef,
  entrypointRef,
  additionalEntrypoints
}: {
  project: string;
  supportedLanguages: string[];
  userRef: RefObject<string | undefined>;
  entrypointRef: RefObject<Entry | undefined>;
  additionalEntrypoints?: string[];
}) => {
  const theme = useAcademyTheme();
  const { warning, error } = useError();
  const filesRef = useRef<Entry[]>([]);
  const runningFilesRef = useRef<JSZip>(JSZip);
  const runningEntrypointRef = useRef<Entry | undefined>(undefined);
  const runningContentRef = useRef<string | undefined>(undefined);
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

  const compareZips = async (zip1: JSZip, zip2: JSZip) => {
    const keys1 = Object.keys(zip1.files);
    const keys2 = Object.keys(zip2.files);
    if (keys1.length !== keys2.length) return false;

    for (const key of keys1) {
      if (!Object.hasOwn(zip2.files, key)) {
        return false;
      }

      if (zip1.files[key].dir && zip2.files[key].dir) continue;
      if (zip1.files[key].dir !== zip2.files[key].dir) return false;

      const value = await zip1.files[key].async("base64");
      const old = await zip2.files[key].async("base64");
      if (value !== old) {
        return false;
      }
    }
    return true;
  };

  const mergeZips = async (zip1: JSZip, zip2: JSZip) => {
    let mergeZip = new JSZip();
    for (const zipObject of [zip1, zip2]) {
      mergeZip = await mergeZip.loadAsync(
        await zipObject.generateAsync({ type: "blob" }),
        { createFolders: true },
      );
    }
    return mergeZip;
  };

  // App handling

  const onAppStateChange = async (save?: boolean): Promise<void> => {
    const manager = CommsManager.getInstance();
    const state = manager.getState();

    setLoading(true);

    if (state === states.WORLD_READY || state === states.CONNECTED) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure a world is selected.",
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
          "Failed to stop the application. See the traces in the terminal.",
        );
      }
      setLoading(false);
      return;
    }

    if (entrypointRef.current === undefined) {
      error(
        "Failed to run the application. Make sure to select an entrypoint by opening it in the editor.",
      );
      setLoading(false);
      return;
    }

    const language = getLanguage(entrypointRef.current.path.split(".").pop());

    if (language === undefined || !supportedLanguages.includes(language)) {
      error(
        `Failed to run the application. Entrypoint ${entrypointRef.current.path} is not supported.`,
      );
      setLoading(false);
      return;
    }

    if (save === undefined) {
      publish("autoSave");
      updateCode(false);
    }

    if (!isCodeUpdatedRef.current) {
      setTimeout(onAppStateChange, 100, true);
      return;
    }

    const files = await getFileList(project, userRef.current);
    filesRef.current = JSON.parse(files);
    const userZip = await loadFiles(
      entrypointRef.current,
      filesRef.current,
      userRef.current,
    );

    if (state === states.PAUSED) {
      const sameZips = await compareZips(userZip, runningFilesRef.current);
      if (sameZips && runningEntrypointRef.current === entrypointRef.current) {
        try {
          await manager.resume();
          console.log("App resumed correctly!");
        } catch (e: unknown) {
          console.error("Error resuming app: " + (e as Error).message);
          error(
            "Failed to resume the application. See the traces in the terminal.",
          );
        }
        setLoading(false);
        return;
      }
    }

    try {
      runningFilesRef.current = userZip;
      const helperZip = new JSZip();
      runningEntrypointRef.current = entrypointRef.current;

      const extension = entrypointRef.current.path.split(".").pop();

      if (extension === "py") {
        await helperZip.loadAsync(commons);
      }

      const helper_files = await getHelperFileList(
        project,
        language ?? "python",
      );

      await zipHelperFiles(
        helperZip,
        helper_files,
        project,
        language ?? "python",
        entrypointRef.current,
      );

      const finalZip = await mergeZips(helperZip, userZip);

      // Convert the blob to base64 using FileReader
      const reader = new FileReader();
      reader.onloadend = async () => {
        const base64data = reader.result; // Get the zip in base64
        // Send the base64 encoded blob
        if (base64data && runningEntrypointRef.current) {
          const entrypoints = [`/workspace/code/${runningEntrypointRef.current.path}`] 
          if (additionalEntrypoints) {
            additionalEntrypoints.forEach(entrypoint => {
              entrypoints.push(`/workspace/code/${entrypoint}`)
            });
          }

          const lint_files = additionalEntrypoints ? additionalEntrypoints : []
          try {
            await manager.run(
              entrypoints,
              [runningEntrypointRef.current.path].concat(lint_files),
              base64data as string,
            );
          } catch {
            error(
              "Failed to run the application. See the traces in the terminal.",
            );
            setLoading(false);
          }
          console.log("Dockerized app started successfully");
        }
      };

      finalZip.generateAsync({ type: "blob" }).then(function (content: Blob) {
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

    async function loadFiles(entrypoint: Entry, files: Entry[], user?: string) {
      const zip = new JSZip();

      await zipCodeFiles(zip, files, project, user);

      zip.files[entrypoint.path].async("string").then(
        (value: string) => (runningContentRef.current = value),
      );
      return zip;
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
