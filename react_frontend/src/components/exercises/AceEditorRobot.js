import * as React from "react";
import { Box } from "@mui/material";
import AceEditor from "react-ace";
import "../../../../static/common/ace-builds/src-noconflict/ace.js";
import "../../../../static/common/ace-builds/src-noconflict/ext-language_tools";
import "../../../../static/common/ace-builds/src-noconflict/mode-python";
import "../../../../static/common/ace-builds/src-noconflict/theme-dracula";
import "../../../../static/common/ace-builds/src-noconflict/snippets/python";
import PropTypes from "prop-types";
import useWindowDimensions from "../../hooks/useWindowDimensions";
export default function AceEditorRobot(props) {
  const [fontSize, setFontSize] = React.useState(14);
  const { editorCode, editorCodeChange } = React.useContext(props.context);

  const editorRef = React.useRef();

  const { height } = useWindowDimensions();

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
      id="code_container"
      sx={{
        m: 1,
        width: "100%",
        flexDirection: "column",
      }}
    >
      <AceEditor
        border="2px solid"
        mode="python"
        theme="dracula"
        name="code"
        width={"100%"}
        height={height * 0.9 + "px"}
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
  );
}

AceEditorRobot.propTypes = {
  context: PropTypes.any,
};
