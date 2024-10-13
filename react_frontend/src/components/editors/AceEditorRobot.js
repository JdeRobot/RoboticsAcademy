import * as React from "react";
import { Box, ButtonGroup, Button } from "@mui/material";
import AddIcon from "@mui/icons-material/Add";
import RemoveIcon from "@mui/icons-material/Remove";

import AceEditor from "react-ace";
import "ace-builds/src-noconflict/ace.js";
import "ace-builds/src-noconflict/ext-language_tools";
import "ace-builds/src-noconflict/mode-python";
import "ace-builds/src-noconflict/theme-dracula";
import "ace-builds/src-noconflict/snippets/python";

import "../../styles/editors/AceEditorRobot.css";

// monaco editor import start
import "../../styles/tailwind.css";
import {
  MonacoEditor,
  EditorTabs,
  editorList,
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

// Rect Components for Monaco
window.RoboticsReactComponentsMonaco =
  window.RoboticsReactComponentsMonaco || {};

window.RoboticsReactComponentsMonaco.CodeEditor = (function () {
  let editorCode = "";
  const editorCodeChangeSuscribers = [];

  //
  let isActive = true;
  const setActive = (active) => (isActive = active);
  const getActive = () => isActive;
  //
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
    //
    setActive: setActive,
    getActive: getActive,
  };
})();

export default function AceEditorRobot(props) {
  // const [activeEditor, setActiveEditor] = React.useState(editorList[1]);
  const [monacoEditorSourceCode, setMonacoEditorSourceCode] = React.useState(
    defaultEditorSourceCode
  );
  //
  const [fontSize, setFontSize] = React.useState(14);
  const [editorCode, setEditorCode] = React.useState(`import GUI
import HAL
# Enter sequential code!

while True:
    # Enter iterative code!`);

  const editorRef = React.useRef();

  const setFontSize_ = (augm) => {
    const ftSize = editorRef.current?.props.fontSize;
    if (augm) {
      if (ftSize < 70) setFontSize(ftSize + 1);
    } else {
      if (ftSize > 2) setFontSize(ftSize - 1);
    }
  };

  const editorCodeChange = (code) => {
    // console.log(`Code changed:\n${code}`);
    setEditorCode(code);
    RoboticsReactComponents.CodeEditor.setCode(code);
  };

  React.useEffect(() => {
    RoboticsReactComponents.CodeEditor.setCode(editorCode);

    RoboticsReactComponents.CodeEditor.OnEditorCodeChanged((code) => {
      // console.log(`Code changed externally to the editor:\n ${code}`);
      setEditorCode(code);
    });

    // monaco
    RoboticsReactComponentsMonaco.CodeEditor.setCode(monacoEditorSourceCode);
    RoboticsReactComponentsMonaco.CodeEditor.OnEditorCodeChanged((code) => {
      setMonacoEditorSourceCode(code);
    });
  }, []);

  //! Monaco Code Editor
  const [state, dispatch] = useEditorReudcer();
  React.useEffect(() => {
    if (state.activeEditor === editorList[0]) {
      RoboticsReactComponentsMonaco.CodeEditor.setActive(false);
    } else RoboticsReactComponentsMonaco.CodeEditor.setActive(true);
  }, [state.activeEditor]);

  // monaco editor code change
  const handleMonacoEditorCodeChange = (code) => {
    setMonacoEditorSourceCode(code);
    RoboticsReactComponentsMonaco.CodeEditor.setCode(code);
  };

  return (
    <Box
      id={state.resizeEditor === "min" ? "code-container" : ""}
      className={`${
        state.resizeEditor === "max"
          ? "absolute left-[8px] top-[144px] z-50 w-[calc(100vw-16px)] h-[calc(100vh-144px)]"
          : ""
      }`}
    >
      <EditorTabs state={state} dispatch={dispatch} />
      {state.activeEditor === editorList[0] ? (
        <>
          <AceEditor
            border="2px solid"
            mode="python"
            theme="dracula"
            name="code"
            width={"100%"}
            height={"100%"}
            onChange={(code) => editorCodeChange(code)}
            ref={editorRef}
            fontSize={fontSize}
            showPrintMargin={true}
            showGutter={true}
            highlightActiveLine={true}
            value={editorCode}
            setOptions={{
              enableBasicAutocompletion: true,
              enableLiveAutocompletion: true,
              enableSnippets: true,
              showLineNumbers: true,
              tabSize: 4,
            }}
          />
          <ButtonGroup variant={"contained"} disableElevation>
            <Button size={"small"} onClick={() => setFontSize_(true)}>
              <AddIcon />
            </Button>
            <Button size={"small"} onClick={() => setFontSize_(false)}>
              <RemoveIcon />
            </Button>
          </ButtonGroup>
        </>
      ) : (
        <div
          className="w-full h-[calc(100%-24px)] border border-slate-600 "
          style={{
            display: state.activeEditor === "monaco" ? "block" : "none",
          }}
        >
          <MonacoEditor
            state={state}
            dispatch={dispatch}
            monacoEditorSourceCode={monacoEditorSourceCode}
            setMonacoEditorSourceCode={setMonacoEditorSourceCode}
            handleMonacoEditorCodeChange={handleMonacoEditorCodeChange}
          />
        </div>
      )}
    </Box>
  );
}
