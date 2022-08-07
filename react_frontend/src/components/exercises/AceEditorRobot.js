import * as React from "react";
import { Box, TextField, Typography } from "@mui/material";
import AceEditor from "react-ace";
import "../../../../static/common/ace-builds/src-noconflict/ace.js";
import "../../../../static/common/ace-builds/src-noconflict/ext-language_tools";
import "../../../../static/common/ace-builds/src-noconflict/mode-python";
import "../../../../static/common/ace-builds/src-noconflict/theme-dracula";
import "../../../../static/common/ace-builds/src-noconflict/snippets/python";
import ExerciseContext from "../../contexts/ExerciseContext";

function AceEditorRobot() {
  const [fontSize, setFontSize] = React.useState(14);
  const {
    filename,
    handleFilenameChange,
    editorCode,
    editorCodeChange,
    editorRef,
  } = React.useContext(ExerciseContext);
  const setFontSize_ = (augm) => {
    const ftSize = editorRef.current?.props.fontSize;
    if (augm) {
      if (ftSize < 70) setFontSize(ftSize + 1);
    } else {
      if (ftSize > 2) setFontSize(ftSize - 1);
    }
  };

  return (
    <Box
      sx={{
        m: 1,
        p: 2,
        flexGrow: 1,
        width: "100%",
        flexDirection: "column",
        border: "2px solid",
      }}
      id="code-control"
    >
      <Box
        sx={{
          m: 1,
          display: "flex",
          justifyContent: "space-between",
          alignItems: "center",
        }}
      >
        <Typography
          align={"center"}
          variant={"h5"}
          sx={{ fontFamily: "Raleway" }}
          color={"secondary"}
        >
          Editor
        </Typography>
        <TextField
          size={"small"}
          id="filename"
          label="Filename"
          color={"secondary"}
          value={filename}
          onChange={handleFilenameChange}
        />
      </Box>
      <Box id="code_container">
        <div id={"editor"}>
          <AceEditor
            mode="python"
            theme="dracula"
            name="code"
            width={"100%"}
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
              tabSize: 2,
            }}
          />
        </div>
        <input
          type="button"
          id="aug_font"
          onClick={() => setFontSize_(true)}
          defaultValue={"+"}
        />
        <input
          type="button"
          id="red_font"
          onClick={() => setFontSize_(false)}
          defaultValue={"-"}
        />
      </Box>
    </Box>
  );
}

export default AceEditorRobot;
