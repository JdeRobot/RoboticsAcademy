import React, { useEffect, useState } from "react";
import LoadingButton from "@mui/lab/LoadingButton";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import PauseIcon from "@mui/icons-material/Pause";
import commons from "./../../common.zip";
import JSZip from "jszip";

declare global {
  interface Window {
    RoboticsExerciseComponents: any;
    RoboticsReactComponents: any;
  }
}


const PlayPauseButton: React.FC = () => {
  const [loading, setLoading] = useState(false);
  const [applicationRunning, setApplicationRunning] = useState(true);
  const [applicationPaused, setApplicationPaused] = useState(false);
  const [disabled, setDisabled] = useState(true);
  const [editorChanged, setEditorChanged] = useState(false);

  
  const commsManager = window.RoboticsExerciseComponents.commsManager;
  const config = JSON.parse(
    document.getElementById("exercise-config")!.textContent || "[]"
  );

  useEffect(() => {
    const callback = (message: any) => {
      const state = message.data.state;
      setApplicationPaused(state === "paused");
      setApplicationRunning(state === "application_running");
      setDisabled(
        !(
          state === "tools_ready" ||
          state === "application_running" ||
          state === "paused"
        )
      );
      setLoading(false);
    };

    commsManager.subscribe([commsManager.events.STATE_CHANGED], callback);

    return () => {
      commsManager.unsubscribe([commsManager.events.STATE_CHANGED], callback);
    };
  }, []);

  useEffect(() => {
    window.RoboticsReactComponents.CodeEditor.OnEditorCodeChanged(() => {
      setEditorChanged(true);
    });
  }, []);

  const play = async () => {
    setLoading(true);
    let editorCode = "";
    editorCode = window.RoboticsReactComponents.CodeEditor.getCode();

    if (applicationPaused) {
      if (editorChanged) {
        await runCode(editorCode);
      }
      commsManager.resume();
    } else {
      await runCode(editorCode);
    }
    setEditorChanged(false);
  };

  const runCode = async (code: string) => {
    setLoading(true);
    const errorMessage =
      "Syntax or dependency error, check details on the console.\n";

    const serverBase = `${document.location.protocol}//${document.location.hostname}:7164`;
    let requestUrl = `${serverBase}/exercises/exercise/${config[0].exercise_id}/user_code_zip`;

    var zip = new JSZip();
    const commonsZip = await zip.loadAsync(commons);
    console.log(commonsZip);

    try {
      const response = await fetch(requestUrl, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
      });

      if (!response.ok) {
        console.error("Error formatting code:");
        return;
      }

      const responseJSON = await response.json();
      const extraFiles: { name: string; content: string }[] = responseJSON.files;

      extraFiles.forEach((file) => {
        commonsZip.file(file.name, file.content);
      });

      commonsZip.file("academy.py", code);

      // add onnx file to the zip if it exists
      if (window.RoboticsReactComponents.DeepLearningModel) {
        const modelBuffer =
          window.RoboticsReactComponents.DeepLearningModel.getModelBuffer();
        if (modelBuffer) {
          commonsZip.file("model.onnx", modelBuffer);
        } else {
          console.warn("No ONNX model buffer found.");
        }
      } else {
        console.warn("DeepLearningModel component not found.");
      }

      // Convert the blob to base64 using FileReader
      const reader = new FileReader();
      reader.onloadend = async () => {
        const base64data = reader.result; // Get the zip in base64
        // Send the base64 encoded blob
        // TODO: temporal until config file
        try {
          await window.RoboticsExerciseComponents.commsManager.run({
            entrypoint: "/workspace/code/academy.py",
            linter: ["academy.py"],
            code: base64data,
          });
        } catch (error) {
          RoboticsReactComponents.MessageSystem.Alert.showAlert(
            errorMessage,
            "error"
          );
        }
      };

      commonsZip.generateAsync({ type: "blob" }).then(function (content) {
        reader.readAsDataURL(content);
      });
    } catch (error) {
      console.log(error);
      return;
    }
  };

  const pause = () => {
    setLoading(true);
    commsManager
      .pause()
      .then(() => {})
      .catch((response: any) => console.log(response))
  };

  return (
    <LoadingButton
      disabled={disabled}
      id={"loadIntoRobot"}
      loading={loading}
      color={"secondary"}
      onClick={applicationRunning ? pause : play}
      sx={{ m: 0.5 }}
      variant={"outlined"}
    >
      {applicationRunning ? <PauseIcon /> : <PlayArrowIcon />}
    </LoadingButton>
  );
};

export default PlayPauseButton;
