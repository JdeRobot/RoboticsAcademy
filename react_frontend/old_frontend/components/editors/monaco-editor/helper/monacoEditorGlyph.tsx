//import * as monaco from "monaco-editor";

// Adds click listener to editor for custom glyph interaction
type GlyphProps = {
  monaco: typeof import("monaco-editor");
  editor: any;
  setLineNumber: (lineNumber: number) => void;
};

export const monacoEditorGlyph = ({ monaco, editor, setLineNumber }: GlyphProps) => {
  editor.onMouseDown((event) => {
    const target = event.target;

    // Check if the click happened in the glyph margin
    if (
      target &&
      target.type === monaco.editor.MouseTargetType.GUTTER_GLYPH_MARGIN
    ) {
      const lineNumber = target.position.lineNumber;

      setLineNumber(lineNumber);
    }
  });
};

export const renderGlyphs = (
  lineNumberDecorationRef: React.MutableRefObject<monaco.editor.IEditorDecorationsCollection | null>,
  allLineNumberDecorations: number[]
) => {
  if (!lineNumberDecorationRef.current || !allLineNumberDecorations) return;
  
  const allGlyphs: monaco.editor.IModelDeltaDecoration[] = allLineNumberDecorations.map(
    (line) => ({
      range: new monaco.Range(line, 1, line, 1),
      options: {
        glyphMarginClassName: "glyph-indicator",
        glyphMarginHoverMessage: {
          value: ``,
        },
      },
    })
  );

  lineNumberDecorationRef.current.set(allGlyphs);
};

type FilterProps = {
  lineNumberDecorations: number[];
  lineNumber: number;
  maxEditorRows: number;
};

// filter line number
export const filterLineNumber = ({
  lineNumberDecorations,
  lineNumber,
  maxEditorRows,
}: FilterProps) => {
  let allLineNumberDecorations = lineNumberDecorations.filter(
    (line) => line <= maxEditorRows
  );

  if (lineNumber === -1) return allLineNumberDecorations;

  // check line number existing
  const lineNumberExisting = allLineNumberDecorations.find(
    (line) => line === lineNumber
  );

  // check if lineNumber already exist, then remove from list and render glyphs
  // otherwise add to list and render glyphs
  if (lineNumberExisting) {
    allLineNumberDecorations = allLineNumberDecorations.filter(
      (line) => line !== lineNumber
    );
  } else allLineNumberDecorations.push(lineNumber);

  return allLineNumberDecorations;
};
