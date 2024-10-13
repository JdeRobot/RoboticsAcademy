import { useRef, useState, useReducer, useEffect } from "react";
import PropTypes from "prop-types";
import Editor, { DiffEditor, useMonaco, loader } from "@monaco-editor/react";
import { monacoEditorScroll } from "./helper/monacoEditorScroll";
import { monacoEditorSnippet } from "./helper/monacoEditorSnippet";
import "./../../../styles/editors/MonacoEditor.css";

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
  const { monacoEditorTheme, editorOptions } = state;
  // USE STATE
  const [decorations, setDecorations] = useState([]);
  const [decorationCollection, setDecorationCollection] = useState(null); // Decoration collection
  const [lineNumber, setLineNumber] = useState(-1);
  const [lineNumberDecorations, setLineNumberDecorations] = useState([]);

  // Editor funcs
  const handleEditorWillMount = (editor, monaco) => {};

  // Trigger formatting on document load
  const handleEditorDidMount = async (editor, monaco) => {
    //TODO: update maxEditor Rows
    // setMaxEditorRows(editorRef.current.getModel().getLineCount());

    // store `useRef`
    monacoRef.current = monaco;
    editorRef.current = editor;
    // Glyphs ref
    lineNumberDecorationRef.current = editor.createDecorationsCollection([]);

    // fontsize/zoom (ctrl+wheel)
    monacoEditorScroll({ editor });

    // editor snippets
    monacoEditorSnippet({ monaco });
  };

  return (
    <div className="w-full h-full">
      <Editor
        height="100%"
        width="100%"
        defaultLanguage="python"
        theme={monacoEditorTheme}
        defaultValue={monacoEditorSourceCode}
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
