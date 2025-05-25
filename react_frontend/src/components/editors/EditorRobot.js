import * as React from "react";
import { Box } from "@mui/material";

import "../../styles/editors/EditorRobot.css";

// monaco editor import start
import "../../styles/tailwind.css";
import {
  MonacoEditor,
  defaultEditorSourceCode,
} from "./monaco-editor";
import { useEditorReudcer } from "../../hooks/useEditorReudcer";
// monaco editor import end

window.RoboticsReactComponents = window.RoboticsReactComponents || {};

window.RoboticsReactComponents.CodeEditor = (function () {
  let editorCode = "";
  const editorCodeChangeSuscribers = [];

  const setCode = (code) => {
    editorCode = code;
    for (
      let i = 0, length = editorCodeChangeSuscribers.length;
      i < length;
      ++i
    ) {
      editorCodeChangeSuscribers[i](code);
    }
  };

  const OnEditorCodeChanged = (handler) => {
    editorCodeChangeSuscribers.push(handler);
  };

  const getCode = () => editorCode;

  return {
    setCode: setCode,
    getCode: getCode,
    OnEditorCodeChanged: OnEditorCodeChanged,
  };
})();

const config = JSON.parse(
  document.getElementById("exercise-config").textContent
);
const exerciseId = config[0].exercise_id;

export default function EditorRobot(props) {
  const [monacoEditorSourceCode, setMonacoEditorSourceCode] = React.useState(
    defaultEditorSourceCode
  );

  React.useEffect(() => {
    // monaco
    RoboticsReactComponents.CodeEditor.OnEditorCodeChanged((code) => {
      setMonacoEditorSourceCode(code);
    });

    let unibotics = undefined;
    try {
      unibotics = props.props.unibotics;
    } catch {}

    if (unibotics) {
      // Request U code
      const request = new Request("/academy/reload_code/" + exerciseId + "/");

      fetch(request)
        .then((response) => {
          if (response.ok) {
            return response.json();
          } else {
            throw new Error("Error getting user code");
          }
        })
        .then((result) => {
          if (result.code != "") {
            const code = result.code
              .replace(/&quot;/g, '"')
              .replace(/&#39;/g, "'")
              .replace(/&gt;/g, ">")
              .replace(/&lt;/g, "<")
              .replace(/&amp;gt;/g, ">")
              .replace(/&amp;lt;/g, "<")
              .replace(/&amp;ge;/g, ">=")
              .replace(/&amp;le;/g, "<=")
              .replace(/&le;/g, "<=")
              .replace(/&ge;/g, ">=")
              .replace(/\\n/g, "\n");
            RoboticsReactComponents.CodeEditor.setCode(code);
          }
        })
        .catch((error) => {
          console.error(error);
        });
    } else {
      RoboticsReactComponents.CodeEditor.setCode(monacoEditorSourceCode);
    }

    const codeLoadedEvent = new CustomEvent("codeLoaded", {
      detail: { isLoading: false },
    });
    window.dispatchEvent(codeLoadedEvent);
  }, []);

  //! Monaco Code Editor
  const [state, dispatch] = useEditorReudcer();

  // monaco editor code change
  const handleMonacoEditorCodeChange = (code) => {
    setMonacoEditorSourceCode(code);
    RoboticsReactComponents.CodeEditor.setCode(code);
  };

  return (
    <Box id="code-container">
      <MonacoEditor
        state={state}
        dispatch={dispatch}
        monacoEditorSourceCode={monacoEditorSourceCode}
        setMonacoEditorSourceCode={setMonacoEditorSourceCode}
        handleMonacoEditorCodeChange={handleMonacoEditorCodeChange}
      />
    </Box>
  );
}
