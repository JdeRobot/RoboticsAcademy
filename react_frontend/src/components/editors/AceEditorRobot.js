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

export default function AceEditorRobot(props) {
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
  }, []);

  return (
    <Box id="code-container">
      <AceEditor
        border="2px solid"
        mode="python"
        theme="dracula"
        name="code"
        width={"100%"}
        height={"100%"}
        onChange={editorCodeChange}
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
    </Box>
  );
}
