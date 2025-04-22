import React, { useEffect, useState } from "react";
import LoadingButton from "@mui/lab/LoadingButton";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import PauseIcon from "@mui/icons-material/Pause";
import JSZip from "jszip";

const PlayPause = (props) => {
  const [loading, setLoading] = useState(false);
  const [applicationRunning, setApplicationRunning] = useState(true);
  const [applicationPaused, setApplicationPaused] = useState(false);
  const [disabled, setDisabled] = useState(true);
  const [editorChanged, setEditorChanged] = useState(false);

  const commsManager = window.RoboticsExerciseComponents.commsManager;
  const config = JSON.parse(
    document.getElementById("exercise-config").textContent
  );

  useEffect(() => {
    const callback = (message) => {
      const state = message.data.state;
      setApplicationPaused(state === "paused");
      setApplicationRunning(state === "application_running");
      setDisabled(
        !(
          state === "visualization_ready" ||
          state === "application_running" ||
          state === "paused"
        )
      );
    };

    commsManager.subscribe([commsManager.events.STATE_CHANGED], callback);

    return () => {
      commsManager.unsubscribe([commsManager.events.STATE_CHANGED], callback);
    };
  }, []);

  React.useEffect(() => {
    RoboticsReactComponents.CodeEditor.OnEditorCodeChanged(() => {
      setEditorChanged(true);
    });
  }, []);

  const play = async () => {
    setLoading(true);
    let editorCode = RoboticsReactComponents.CodeEditor.getCode();

    if (applicationPaused) {
      if (editorChanged) {
        await runCode(editorCode);
      }
      commsManager.resume();
    } else {
      await runCode(editorCode);
    }
    setLoading(false);
    setEditorChanged(false);
  };

  const runCode = async (code) => {
    setLoading(true);
    const errorMessage =
      "Syntax or dependency error, check details on the console.\n";

    const serverBase = `${document.location.protocol}//${document.location.hostname}:7164`;
    let requestUrl = `${serverBase}/exercises/exercise/${config[0].exercise_id}/user_code_zip`;

    const zip = new JSZip();

    try {
      const commonsResponse = await fetch("/common.zip");
      if (!commonsResponse.ok) throw new Error("Failed to load common.zip");

      const commonsBlob = await commonsResponse.blob();
      const commonsZip = await zip.loadAsync(commonsBlob);

      const response = await fetch(requestUrl, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
      });

      if (!response.ok) {
        console.error("Error fetching extra files:", response.statusText);
        return;
      }

      const responseJSON = await response.json();
      const extraFiles = responseJSON.files;

      extraFiles.forEach((file) => {
        commonsZip.file(file.name, file.content);
      });

      commonsZip.file("academy.py", code);

      const reader = new FileReader();
      reader.onloadend = async () => {
        const base64data = reader.result;
        try {
          await commsManager.run({
            type: "robotics-academy",
            code: base64data,
          });
        } catch (error) {
          RoboticsReactComponents.MessageSystem.Alert.showAlert(
            errorMessage,
            "error"
          );
        }
      };

      commonsZip.generateAsync({ type: "blob" }).then((content) => {
        reader.readAsDataURL(content);
      });
    } catch (error) {
      console.error("Run code error:", error);
      RoboticsReactComponents.MessageSystem.Alert.showAlert(
        "Failed to prepare and send user code.",
        "error"
      );
    }
  };

  const pause = () => {
    setLoading(true);
    commsManager
      .pause()
      .catch((response) => console.log(response))
      .finally(() => setLoading(false));
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

export default PlayPause;

