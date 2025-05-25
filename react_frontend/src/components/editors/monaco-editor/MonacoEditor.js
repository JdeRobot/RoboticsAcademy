import { useEffect, useRef, useState } from "react";
import Editor, { loader } from "@monaco-editor/react";
import PropTypes from "prop-types";
import {
  MonacoEditorLoader,
  monacoEditorSnippet,
  monacoEditorGlyph,
  monacoEditorScroll,
  setEditorSettingsWidgetsData,
  getEditorSettingsWidgetsData,
} from "./index";
import {
  useMonacoEditorLoaderEffect,
  useMonacoEditorCodeAnalysisEffect,
  useMonacoEditorCodeFormatEffect,
  useMonacoEditorLineNumberDecorationsEffect,
} from "../../../hooks/useMonacoEditorEffect";
import "./../../../styles/editors/MonacoEditor.css";
import MonacoEditorInfo from "./editor-info/MonacoEditorInfo";

const MonacoEditor = ({
  state,
  dispatch,
  monacoEditorSourceCode,
  setMonacoEditorSourceCode,
  handleMonacoEditorCodeChange,
}) => {
  // USE Ref
  const monacoRef = useRef(null);
  const editorRef = useRef(null);
  const lineNumberDecorationRef = useRef(null);
  // Rducer state
  const {
    isLoading,
    monacoEditorTheme,
    editorOptions,
    editorSettings,
  } = state;
  // USE STATE
  const [lineNumber, setLineNumber] = useState(-1);
  const [lineNumberDecorations, setLineNumberDecorations] = useState([]);
  const [updateGlyphs, setUpdateGlyphs] = useState(false);
  const [maxEditorRows, setMaxEditorRows] = useState(-1);

  // USE Effects
  //localstorage
  useEffect(() => {
    const data = getEditorSettingsWidgetsData();

    if (data) {
      dispatch({
        type: "udpateEditorSttingsWidgets",
        payload: {
          isCodeFormatEnable: data.isCodeFormatEnable,
          isZoomingEnable: data.isZoomingEnable,
        },
      });
    } else {
      setEditorSettingsWidgetsData({
        isCodeFormatEnable: editorSettings.isCodeFormatEnable,
        isZoomingEnable: editorSettings.isZoomingEnable,
      });
    }
  }, []);
  // called when widgets changed
  useEffect(() => {
    setEditorSettingsWidgetsData({
      isCodeFormatEnable: editorSettings.isCodeFormatEnable,
      isZoomingEnable: editorSettings.isZoomingEnable,
    });
  }, [editorSettings.isCodeFormatEnable, editorSettings.isZoomingEnable]);

  // editor loading
  useMonacoEditorLoaderEffect({ loader, dispatch, monacoEditorTheme });

  // UseEffect for Line Number Decorations
  useMonacoEditorLineNumberDecorationsEffect({
    editorRef,
    lineNumberDecorationRef,
    setLineNumber,
    lineNumber,
    updateGlyphs,
    setUpdateGlyphs,
    monacoEditorSourceCode,
    setLineNumberDecorations,
    lineNumberDecorations,
    maxEditorRows,
    setMaxEditorRows,
  });
  // code analysis (pylint)
  useMonacoEditorCodeAnalysisEffect({
    monacoRef,
    editorRef,
    monacoEditorSourceCode,
  });

  // Code format (black)
  useMonacoEditorCodeFormatEffect({
    editorRef,
    monacoEditorSourceCode,
    setMonacoEditorSourceCode,
    setUpdateGlyphs,
  });

  // Editor funcs
  const handleEditorWillMount = (editor, monaco) => {};

  // Trigger formatting on document load
  const handleEditorDidMount = async (editor, monaco) => {
    // store `useRef`
    monacoRef.current = monaco;
    editorRef.current = editor;
    // Glyphs ref
    lineNumberDecorationRef.current = editor.createDecorationsCollection([]);

    // update maxEditor Rows
    setMaxEditorRows(editorRef.current.getModel().getLineCount());

    // fontsize/zoom (ctrl+wheel)
    monacoEditorScroll({ editor });

    // editor snippets
    monacoEditorSnippet({ monaco });

    // Glyphs
    monacoEditorGlyph({ monaco, editor, setLineNumber });
  };

  return (
    <div className="w-full h-full">
      {isLoading ? (
        <MonacoEditorLoader theme={monacoEditorTheme} />
      ) : (
        <>
          <MonacoEditorInfo
            editorSettings={editorSettings}
            dispatch={dispatch}
            editorRef={editorRef}
          />
          <Editor
            height="100%"
            width="100%"
            defaultLanguage="python"
            theme={monacoEditorTheme}
            defaultValue={monacoEditorSourceCode}
            value={monacoEditorSourceCode}
            onChange={(code) => handleMonacoEditorCodeChange(code)}
            beforeMount={handleEditorWillMount}
            onMount={handleEditorDidMount}
            // Editor Options
            options={editorOptions}
            className=""
          />
        </>
      )}
    </div>
  );
};

export default MonacoEditor;

MonacoEditor.prototype = {
  state: PropTypes.object.isRequired,
  dispatch: PropTypes.func.isRequired,
  monacoEditorSourceCode: PropTypes.string.isRequired,
  setMonacoEditorSourceCode: PropTypes.func.isRequired,
  handleMonacoEditorCodeChange: PropTypes.func.isRequired,
};
