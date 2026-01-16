import { StyledHeaderButton } from "Styles/headers/HeaderMenu.styles";
import { useError } from "jderobot-ide-interface";
import { publish, subscribe, unsubscribe } from "Helpers/utils";
import { CommsManager, states } from "jderobot-commsmanager";
import JSZip from "jszip";
import { useExercise } from "Contexts/ExerciseContext";
import { useEffect, useRef, useState } from "react";
import commons from "../../common.zip";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import React from "react";

import PlayArrowRoundedIcon from "@mui/icons-material/PlayArrowRounded";
import PauseRoundedIcon from "@mui/icons-material/PauseRounded";
import SyncRoundedIcon from "@mui/icons-material/SyncRounded";
import { getProjectExtraFiles } from "Api";

const PlayPauseButton = ({
  project,
  language,
  dlModel,
  hasDLModel,
  connectManager,
}: {
  project: string;
  language?: string;
  dlModel: ArrayBuffer | undefined;
  hasDLModel: boolean;
  connectManager: (
    desiredState?: string,
    callback?: () => void
  ) => Promise<void>;
}) => {
  const theme = useAcademyTheme();
  const exerciseContext = useExercise();
  const { warning, error, info, close } = useError();
  const codeRef = useRef("");
  const runningCodeRef = useRef("");
  const runningDLModel = useRef<ArrayBuffer | undefined>(undefined);
  const [state, setState] = useState<string>(
    CommsManager.getInstance().getState()
  );
  const [loading, setLoading] = useState<boolean>(false);
  const isCodeUpdatedRef = useRef<boolean | undefined>(undefined);
  const [isCodeUpdated, _updateCode] = useState<boolean | undefined>(false);

  const updateCode = (data?: boolean) => {
    isCodeUpdatedRef.current = data;
    _updateCode(data);
  };

  const updateState = (e: any) => {
    setState(e.detail.state);
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
    codeRef.current = exerciseContext.code;
  }, [exerciseContext]);

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

    if (state === states.PAUSED) {
      if (
        runningCodeRef.current === codeRef.current &&
        runningDLModel.current === dlModel
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

      const extraFiles: { name: string; content: string }[] =
        await getProjectExtraFiles(project, language ? language : "python");

      extraFiles.forEach((file) => {
        commonsZip.file(file.name, file.content);
      });

      commonsZip.file(`academy.${extension}`, codeRef.current);

      // add onnx file to the zip if it exists
      if (hasDLModel) {
        if (dlModel !== undefined) {
          commonsZip.file("model.onnx", dlModel);
        } else {
          throw new Error("No ONNX model found.");
        }
      }

      runningCodeRef.current = codeRef.current;

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

      zip.generateAsync({ type: "blob" }).then(function (content: Blob) {
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
