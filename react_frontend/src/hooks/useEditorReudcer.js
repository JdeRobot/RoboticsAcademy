import { useReducer } from "react";
import {
  monacoEditorThemeList,
  resizeList,
} from "../components/editors/monaco-editor";

const editorInitialState = {
  isLoading: true,
  monacoEditorTheme: monacoEditorThemeList[0], //
  resizeEditor: resizeList[0],
  editorOptions: {
    //
    fontSize: 14,
    lineNumbers: "on",
    roundedSelection: false,
    scrollBeyondLastLine: true,
    // word warp
    wordWrap: "wordWrapColumn",
    wordWrapColumn: 80,
    wrappingIndent: "indent",
    //
    minimap: { enabled: false },
    automaticLayout: true,
    tabSize: 4,
    rulers: [80],
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
  editorSettings: {
    isModalOpen: false,
    isCodeFormatEnable: true,
    isZoomingEnable: true,
    modalScreenState: "shortcuts", // widgets
  },
};

const reducer = (state, action) => {
  switch (action.type) {
    // editor options
    case "updateEditorState":
      return {
        ...state,
        isLoading: action.payload.loading,
      };
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
        editorOptions: {
          ...state.editorOptions,
          minimap: { enabled: state.resizeEditor === "min" },
        },
      };
    case "updateMonacoEditorTheme":
      return {
        ...state,
        monacoEditorTheme: action.payload.theme,
      };

    // editor settings
    case "udpateEditorSttingsWidgets":
      return {
        ...state,
        editorSettings: {
          ...state.editorSettings,
          isCodeFormatEnable: action.payload.isCodeFormatEnable,
          isZoomingEnable: action.payload.isZoomingEnable,
        },
      };
    case "changeSettingsModalState":
      return {
        ...state,
        editorSettings: {
          ...state.editorSettings,
          isModalOpen: action.payload.isModalOpen,
        },
      };
    case "changeModalScreenState":
      return {
        ...state,
        editorSettings: {
          ...state.editorSettings,
          modalScreenState: action.payload.screen,
        },
      };
    case "isCodeFormatEnable":
      return {
        ...state,
        editorSettings: {
          ...state.editorSettings,
          isCodeFormatEnable: !state.editorSettings.isCodeFormatEnable,
        },
      };
    case "isZoomingEnable":
      return {
        ...state,
        editorSettings: {
          ...state.editorSettings,
          isZoomingEnable: !state.editorSettings.isZoomingEnable,
        },
      };
    default:
      throw new Error("Unknown Action type!");
  }
};

export const useEditorReudcer = () => {
  const [state, dispatch] = useReducer(reducer, editorInitialState);

  return [state, dispatch];
};
