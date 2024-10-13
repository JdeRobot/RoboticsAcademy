import { useRef, useState, useReducer, useEffect } from "react";
import PropTypes from "prop-types";
import Editor, { DiffEditor, useMonaco, loader } from "@monaco-editor/react";
import { monacoEditorScroll } from "./helper/monacoEditorScroll";
import { monacoEditorSnippet } from "./helper/monacoEditorSnippet";
import "./../../../styles/editors/MonacoEditor.css";
import { filterLineNumber, monacoEditorGlyph, renderGlyphs } from "./index";

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
  const [updateGlyphs, setUpdateGlyphs] = useState(false);
  const [maxEditorRows, setMaxEditorRows] = useState(-1);

  // USE Effects
  // UseEffect for Line Number Decorations
  useEffect(() => {
    if (lineNumber === -1 && !updateGlyphs) return;

    let allLineNumberDecorations = filterLineNumber({
      lineNumberDecorations,
      lineNumber,
      maxEditorRows,
    });

    console.log("====================================");
    console.log("maxEditorRows ", maxEditorRows);
    console.log("lineNumberDecorations ", lineNumberDecorations);
    console.log("allLineNumberDecorations ", allLineNumberDecorations);
    console.log("====================================");

    setLineNumberDecorations(allLineNumberDecorations);
    // reset lineNumber
    setLineNumber(-1);
    setUpdateGlyphs(false);

    // render Glyphs
    renderGlyphs(lineNumberDecorationRef, allLineNumberDecorations);
  }, [lineNumber, updateGlyphs]);

  useEffect(() => {
    if (editorRef.current) {
      const model = editorRef.current.getModel();
      const lineCount = model.getLineCount();

      setMaxEditorRows((prev) => {
        if (prev > lineCount) {
          setUpdateGlyphs(true);
        }
        return lineCount;
      });
    }
  }, [monacoEditorSourceCode]);

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
    // editor.onMouseDown((event) => {
    //   const target = event.target;

    //   // Check if the click happened in the glyph margin
    //   if (
    //     target &&
    //     target.type === monaco.editor.MouseTargetType.GUTTER_GLYPH_MARGIN
    //   ) {
    //     const lineNumber = target.position.lineNumber;

    //     setLineNumber(lineNumber);
    //   }
    // });
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
