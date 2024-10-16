import { useEffect } from "react";
import {
  fetchAnalysisCode,
  fetchFormatCode,
  filterLineNumber,
  getMarkerSeverity,
  renderGlyphs,
} from "../components/editors/monaco-editor";

// Code Analysis (with pylint)
export const useMonacoEditorCodeAnalysisEffect = ({
  baseUrl,
  monacoRef,
  editorRef,
  monacoEditorSourceCode,
}) => {
  useEffect(() => {
    if (!editorRef.current || !monacoRef.current) return;

    const controller = new AbortController();

    const drawMarker = async () => {
      const data = await fetchAnalysisCode({
        baseUrl,
        monacoEditorSourceCode,
        controller,
      });

      if (!data) return;

      const model = editorRef.current.getModel();
      const pylint_data = data.pylint_output.map((pylint, i) => {
        return {
          startLineNumber: pylint.line,
          startColumn: pylint.column,
          endLineNumber:
            pylint.endLine === null ? pylint.column : pylint.endLine,
          endColumn:
            pylint.endColumn === null
              ? model.getLineMaxColumn(pylint.line)
              : pylint.endColumn,
          message: pylint.message,
          severity: getMarkerSeverity({
            type: pylint.type,
            monaco: monacoRef.current,
          }),
        };
      });
      monacoRef.current.editor.setModelMarkers(model, "owner", pylint_data);
    };

    drawMarker();

    return () => controller.abort();
  }, [monacoEditorSourceCode]);
};

// Code Format (with black)
export const useMonacoEditorCodeFormatEffect = ({
  baseUrl,
  monacoEditorSourceCode,
  setMonacoEditorSourceCode,
  setUpdateGlyphs,
}) => {
  // Use Effect for Black (code prettify/beautify)
  useEffect(() => {
    const handleKeyDown = async (event) => {
      // Check if Ctrl+S is pressed
      if (event.ctrlKey && (event.key === "s" || event.key === "S")) {
        event.preventDefault();

        try {
          await fetchFormatCode({
            baseUrl,
            setMonacoEditorSourceCode,
            monacoEditorSourceCode,
          });
          setUpdateGlyphs(true);
        } catch (error) {
          console.log(error);
        }
      }
    };

    window.addEventListener("keydown", handleKeyDown);

    // Clean up event
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
    };
  }, [monacoEditorSourceCode]);
};

export const useMonacoEditorLineNumberDecorationsEffect = ({
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
}) => {
  useEffect(() => {
    if (
      (lineNumber === -1 && !updateGlyphs) ||
      !lineNumberDecorationRef.current ||
      !editorRef.current
    )
      return;

    let allLineNumberDecorations = filterLineNumber({
      lineNumberDecorations,
      lineNumber,
      maxEditorRows,
    });

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
};
