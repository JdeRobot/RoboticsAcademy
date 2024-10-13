import { useReducer } from "react";
import {
  editorList,
  monacoEditorTheme,
} from "../components/editors/monaco-editor";

const editorInitialState = {
  activeEditor: editorList[1], // monaco
  monacoEditorTheme: monacoEditorTheme[1], // vs-dark
  editorOptions: {
    //
    fontSize: 14,
    lineNumbers: "on",
    roundedSelection: false,
    scrollBeyondLastLine: false,
    readOnly: false, //F: Lock editor
    // word warp
    wordWrap: "wordWrapColumn",
    wordWrapColumn: 100,
    wrappingIndent: "indent",
    //
    minimap: { enabled: false },
    automaticLayout: true,
    tabSize: 4,
    rulers: [],
    suggestOnTriggerCharacters: true,
    quickSuggestions: true,
    wordBasedSuggestions: true,
    //
    hover: true,
    glyphMargin: true, // Enable the glyph margin
    lineNumbersMinChars: 3,
    // scroll
    smoothScrolling: true,
    scrollbar: {
      vertical: "auto",
      horizontal: "auto",
      verticalScrollbarSize: 8,
      horizontalScrollbarSize: 8,
    },
  },
};

const reducer = ({ state, action }) => {
  switch (action.type) {
    case "changeEditor":
      return {
        ...state,
        activeEditor: action.payload.editor,
      };

    default:
      throw new Error("Unknown Action type!");
  }
};

export const useEditorReudcer = () => {
  const [state, dispatch] = useReducer(reducer, editorInitialState);

  return [state, dispatch];
};
