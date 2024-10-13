import { useReducer } from "react";
import {
  editorList,
  monacoEditorThemeList,
  resizeList,
} from "../components/editors/monaco-editor";

const editorInitialState = {
  activeEditor: editorList[1], // monaco
  monacoEditorTheme: monacoEditorThemeList[1], // vs-dark
  resizeEditor: resizeList[0],
  editorOptions: {
    //
    fontSize: 14,
    lineNumbers: "on",
    roundedSelection: false,
    scrollBeyondLastLine: true,
    readOnly: false,
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
    glyphMargin: true, 
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

const reducer = (state, action) => {
  switch (action.type) {
    case "changeEditor":
      return {
        ...state,
        activeEditor: action.payload.editor,
      };
    case "updateEditorResize":
      return {
        ...state,
        resizeEditor:
          state.resizeEditor === resizeList[0] ? resizeList[1] : resizeList[0],
      };
    case "updateLockEditor":
      return {
        ...state,
        editorOptions: {
          ...state.editorOptions,
          readOnly: !state.editorOptions.readOnly,
        },
      };
    case "updateMonacoEditorTheme":
      return{
        ...state,
        monacoEditorTheme:action.payload.theme
      }
    default:
      throw new Error("Unknown Action type!");
  }
};

export const useEditorReudcer = () => {
  const [state, dispatch] = useReducer(reducer, editorInitialState);

  return [state, dispatch];
};
