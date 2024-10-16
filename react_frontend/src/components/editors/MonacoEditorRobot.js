import * as React from "react";
import { Box, ButtonGroup, Button } from "@mui/material";
import AddIcon from "@mui/icons-material/Add";
import RemoveIcon from "@mui/icons-material/Remove";
import Editor, { loader } from "@monaco-editor/react"; 
import "../../styles/editors/MonacoEditorRobot.css";

import {createDependencyProposals} from "./MonacoEditorSnippets";

window.RoboticsReactComponents = window.RoboticsReactComponents || {};

window.RoboticsReactComponents.CodeEditor = (function () {
  let editorCode = "";
  const editorCodeChangeSubscribers = [];

  const setCode = (code) => {
    editorCode = code;
    for (let i = 0, length = editorCodeChangeSubscribers.length; i < length; ++i) {
      editorCodeChangeSubscribers[i](code);
    }
  };

  const OnEditorCodeChanged = (handler) => {
    editorCodeChangeSubscribers.push(handler);
  };

  const getCode = () => editorCode;

  return {
    setCode,
    getCode,
    OnEditorCodeChanged,
  };
})();

export default function MonacoEditorRobot(props) {
  const [fontSize, setFontSize] = React.useState(14);
  const [editorCode, setEditorCode] = React.useState(`import GUI
import HAL
# Enter sequential code!

while True:
    # Enter iterative code!`);

  const editorRef = React.useRef();

  const setFontSize_ = (augm) => {
    if (augm && fontSize < 70) {
      setFontSize(fontSize + 1);
    } else if (!augm && fontSize > 2) {
      setFontSize(fontSize - 1);
    }
  };

  const editorCodeChange = (code) => {
    setEditorCode(code);
    RoboticsReactComponents.CodeEditor.setCode(code);
  };

  React.useEffect(() => {
    loader.init().then((monaco) => {
      console.log("Monaco Editor loaded: ", monaco);

      monaco.languages.registerCompletionItemProvider("python", {
        provideCompletionItems: function (model, position) {
          // find out if we are completing a property in the 'dependencies' object.
          var textUntilPosition = model.getValueInRange({
            startLineNumber: 1,
            startColumn: 1,
            endLineNumber: position.lineNumber,
            endColumn: position.column,
          });
          // var match = textUntilPosition.match(
          //   /"dependencies"\s*:\s*\{\s*("[^"]*"\s*:\s*"[^"]*"\s*,\s*)*([^"]*)?$/
          // );
          // if (!match) {
          //   return { suggestions: [] };
          // }
          var word = model.getWordUntilPosition(position);
          var range = {
            startLineNumber: position.lineNumber,
            endLineNumber: position.lineNumber,
            startColumn: word.startColumn,
            endColumn: word.endColumn,
          };
          return {
            suggestions: createDependencyProposals(range),
          };
        },
      });
    }).catch((error) => {
      console.error("Error loading Monaco Editor: ", error);
    });
  }, []); 

  React.useEffect(() => {
    RoboticsReactComponents.CodeEditor.setCode(editorCode);
    RoboticsReactComponents.CodeEditor.OnEditorCodeChanged((code) => {
      setEditorCode(code);
    });
  }, []);

  return (
    <Box id="code-container">
      <Editor
        height="100%"
        width="100%"
        language="python"
        theme="vs-dark" 
        value={editorCode}
        options={{
          fontSize: fontSize,
          minimap: { enabled: false },
          automaticLayout: true,
          scrollBeyondLastLine: true,
          wordWrap: "on",
          tabSize: 4,
          rulers: [80],
          suggestOnTriggerCharacters: true,
          quickSuggestions: true,
          wordBasedSuggestions: true,
        }}
        onChange={editorCodeChange}
        editorDidMount={(editor) => {
          editorRef.current = editor;
          console.log("Editor mounted: ", editor);
        }}
      />
      <ButtonGroup variant="contained" disableElevation>
        <Button size="small" onClick={() => setFontSize_(true)}>
          <AddIcon />
        </Button>
        <Button size="small" onClick={() => setFontSize_(false)}>
          <RemoveIcon />
        </Button>
      </ButtonGroup>
    </Box>
  );
}
