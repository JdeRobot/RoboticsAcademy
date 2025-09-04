import { StyledHeaderButton } from "Components/headers/HeaderMenu.styles";
import { useError, useTheme } from "jderobot-ide-interface";
import { publish, subscribe, unsubscribe } from "Helpers/utils";
import { CommsManager } from "jderobot-commsmanager";
import { getProjectExtraFiles } from "Helpers/api";
import JSZip from "jszip";
import { useExercise } from "Contexts/ExerciseContext";
import { useEffect, useRef, useState } from "react";
import commons from "../../common.zip";

import PlayArrowRoundedIcon from "@mui/icons-material/PlayArrowRounded";
import PauseRoundedIcon from "@mui/icons-material/PauseRounded";

const PlayPauseButton = ({
  project,
  manager,
  appRunning,
  setAppRunning,
  dlModel,
  hasDLModel,
}: {
  project: string;
  manager: CommsManager | null;
  appRunning: boolean;
  setAppRunning: Function;
  dlModel: string;
  hasDLModel: boolean;
}) => {
  const theme = useTheme();
  const exerciseContext = useExercise();
  const { warning, error } = useError();
  const codeRef = useRef("");
  const runningCodeRef = useRef("");
  const isCodeUpdatedRef = useRef<boolean | undefined>(undefined);
  const [isCodeUpdated, _updateCode] = useState<boolean | undefined>(false);

  const updateCode = (data?: boolean) => {
    isCodeUpdatedRef.current = data;
    _updateCode(data);
  };

  useEffect(() => {
    subscribe("autoSaveCompleted", () => {
      updateCode(true);
    });

    return () => {
      unsubscribe("autoSaveCompleted", () => {});
    };
  }, []);

  useEffect(() => {
    codeRef.current = exerciseContext.code;
  }, [exerciseContext]);

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

    if (appRunning) {
      try {
        await manager.pause();
        setAppRunning(false);
        console.log("App paused correctly!");
        return;
      } catch (e: unknown) {
        if (e instanceof Error) {
          console.error("Error pausing app: " + e.message);
          error("Error pausing app: " + e.message);
        }
      }
    }

    if (save === undefined) {
      publish("autoSave");
      updateCode(false);
    }

    if (!isCodeUpdatedRef.current) {
      return setTimeout(onAppStateChange, 100, true);
    }

    if (runningCodeRef.current === codeRef.current) {
      await manager.resume();
      setAppRunning(true);
      console.log("App resumed correctly!");
      return;
    }

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

      runningCodeRef.current = codeRef.current;

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

      zip.generateAsync({ type: "blob" }).then(function (content: Blob) {
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
  };

  return (
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
  );
};

export default PlayPauseButton;
