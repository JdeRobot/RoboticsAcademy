import { useRef, useState, useReducer, useEffect } from "react";
import PropTypes from "prop-types";
import Editor, { DiffEditor, useMonaco, loader } from "@monaco-editor/react";
import { monacoEditorScroll } from "./helper/monacoEditorScroll";
import { monacoEditorSnippet } from "./helper/monacoEditorSnippet";
import "./../../../styles/editors/MonacoEditor.css";
import { getHalGuiMethods, monacoEditorGlyph } from "./index";
import {
  useMonacoEditorCodeAnalysisEffect,
  useMonacoEditorCodeFormatEffect,
  useMonacoEditorLineNumberDecorationsEffect,
} from "../../../hooks/useMonacoEditorEffect";

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
  //
  const { monacoEditorTheme, editorOptions, baseUrl } = state;
  // USE STATE
  const [lineNumber, setLineNumber] = useState(-1);
  const [lineNumberDecorations, setLineNumberDecorations] = useState([]);
  const [updateGlyphs, setUpdateGlyphs] = useState(false);
  const [maxEditorRows, setMaxEditorRows] = useState(-1);

  // USE Effects

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
    baseUrl,
    monacoRef,
    editorRef,
    monacoEditorSourceCode,
  });

  // Code format (black)
  useMonacoEditorCodeFormatEffect({
    baseUrl,
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
    const { guiAutoComplete, halAutoComplete } = getHalGuiMethods({ monaco });

    monacoEditorSnippet({ monaco, guiAutoComplete, halAutoComplete });

    // Glyphs
    monacoEditorGlyph({ monaco, editor, setLineNumber });
  };

  return (
    <div className="w-full h-full">
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
        // onValidate={handleEditorValidation}
        // Editor Options
        options={editorOptions}
        className=""
      />
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
