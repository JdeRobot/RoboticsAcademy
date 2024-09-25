import * as React from "react";
import { Box, ButtonGroup, Button } from "@mui/material";
import AddIcon from "@mui/icons-material/Add";
import RemoveIcon from "@mui/icons-material/Remove";
import Editor from "@monaco-editor/react";
import "../../styles/editors/MonacoEditorRobot.css";

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
        theme="vs-dark" // Change the theme here if needed
        value={editorCode}
        options={{
          fontSize: fontSize,
          minimap: { enabled: false },
          automaticLayout: true,
          scrollBeyondLastLine: false,
          wordWrap: "on",
          tabSize: 4,
        }}
        onChange={editorCodeChange} // Simplified onChange
        editorDidMount={(editor) => {
          editorRef.current = editor;
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
