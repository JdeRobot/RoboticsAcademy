import * as React from "react";
import { Box } from "@mui/material";

import "../../styles/editors/EditorRobot.css";

// monaco editor import start
import "../../styles/tailwind.css";
import { defaultEditorSourceCode } from "./monaco-editor/constants";
import MonacoEditor from "./monaco-editor/MonacoEditor";
import { useEditorReudcer } from "../../hooks/useEditorReudcer";
// monaco editor import end

declare global {
  interface Window {
    RoboticsReactComponents: {
      CodeEditor: {
        setCode: (code: string) => void;
        getCode: () => string;
        OnEditorCodeChanged: (handler: (code: string) => void) => void;
      };
    };
  }
}

window.RoboticsReactComponents = window.RoboticsReactComponents || {};

window.RoboticsReactComponents.CodeEditor = window.RoboticsReactComponents.CodeEditor || (function () {
  let editorCode = "";
  const editorCodeChangeSubscribers: ((code: string) => void)[] = [];

  const setCode = (code: string) => {
    editorCode = code;
    for (
      let i = 0, length = editorCodeChangeSubscribers.length;
      i < length;
      ++i
    ) {
      editorCodeChangeSubscribers[i](code);
    }
  };

  const OnEditorCodeChanged = (handler: (code: string) => void) => {
    editorCodeChangeSubscribers.push(handler);
  };

  const getCode = () => editorCode;

  return {
    setCode: setCode,
    getCode: getCode,
    OnEditorCodeChanged: OnEditorCodeChanged,
  };
})();

const configText = document.getElementById("exercise-config")?.textContent || "[]";
const config = JSON.parse(configText);
const exerciseId = config?.[0]?.exercise_id ?? "";

interface EditorRobotProps {
  props?: {
    unibotics?: boolean;
  };
}

export default function EditorRobot({props}: EditorRobotProps) {
  const [monacoEditorSourceCode, setMonacoEditorSourceCode] = React.useState<string>(
    defaultEditorSourceCode
  );

  React.useEffect(() => {
    // monaco
    RoboticsReactComponents.CodeEditor.OnEditorCodeChanged((code) => {
      setMonacoEditorSourceCode(code);
    });

    let unibotics = undefined;
    try {
      unibotics = Boolean(props?.unibotics);
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

     window.dispatchEvent(
      new CustomEvent("codeLoaded", {
        detail: { isLoading: false },
      })
    );
  }, []);

  //! Monaco Code Editor
  const [state, dispatch] = useEditorReudcer();

  // monaco editor code change
  const handleMonacoEditorCodeChange = (code: string | undefined) => {
    const newCode = code || "";
    setMonacoEditorSourceCode(newCode);
    RoboticsReactComponents.CodeEditor.setCode(newCode);
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
