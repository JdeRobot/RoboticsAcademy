import * as React from "react";
import { Box, ButtonGroup, Button } from "@mui/material";
import AddIcon from "@mui/icons-material/Add";
import RemoveIcon from "@mui/icons-material/Remove";
import Editor, { loader } from "@monaco-editor/react"; 
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
    loader.init().then((monaco) => {
      console.log("Monaco Editor loaded: ", monaco);

      monaco.languages.registerCompletionItemProvider("python", {
        provideCompletionItems: (model, position) => {
          const textUntilPosition = model.getValueInRange({
            startLineNumber: 1,
            startColumn: 1,
            endLineNumber: position.lineNumber,
            endColumn: position.column,
          });

          const suggestions = [];
          if (textUntilPosition.endsWith("for")) {
            suggestions.push({
              label: "for",
              kind: monaco.languages.CompletionItemKind.Snippet,
              insertText: "for ${1:item} in ${2:iterable}:\n\t$0",
              insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
              documentation: "Snippet for a for loop",
            });
          } else if (textUntilPosition.endsWith("while")) {
            suggestions.push({
              label: "while",
              kind: monaco.languages.CompletionItemKind.Snippet,
              insertText: "while ${1:condition}:\n\t$0",
              insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
              documentation: "Snippet for a while loop",
            });
          } else if (textUntilPosition.endsWith("if")) {
            suggestions.push({
              label: "if",
              kind: monaco.languages.CompletionItemKind.Snippet,
              insertText: "if ${1:condition}:\n\t$0",
              insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
              documentation: "Snippet for an if statement",
            });
          } else if (textUntilPosition.endsWith("elif")) {
            suggestions.push({
              label: "elif",
              kind: monaco.languages.CompletionItemKind.Snippet,
              insertText: "elif ${1:condition}:\n\t$0",
              insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
              documentation: "Snippet for an elif statement",
            });
          } else if (textUntilPosition.endsWith("print")) {
            suggestions.push({
              label: "print",
              kind: monaco.languages.CompletionItemKind.Snippet,
              insertText: "print(${1:message})",
              insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
              documentation: "Snippet for a print statement",
            });
          } else if (textUntilPosition.endsWith("def")) {
            suggestions.push({
              label: "def",
              kind: monaco.languages.CompletionItemKind.Snippet,
              insertText: "def ${1:function_name}(${2:params}):\n\t$0",
              insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
              documentation: "Snippet for a function definition",
            });
          } else if (textUntilPosition.endsWith("try")) {
            suggestions.push({
              label: "try",
              kind: monaco.languages.CompletionItemKind.Snippet,
              insertText: "try:\n\t$0\nexcept ${1:Exception} as ${2:e}:\n\tpass",
              insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
              documentation: "Snippet for try-except block",
            });
          } else if (textUntilPosition.endsWith("class")) {
            suggestions.push({
              label: "class",
              kind: monaco.languages.CompletionItemKind.Snippet,
              insertText: "class ${1:ClassName}(${2:object}):\n\tdef __init__(self, ${3:args}):\n\t\t$0",
              insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
              documentation: "Snippet for a class definition",
            });
          }

          return { suggestions: suggestions };
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
