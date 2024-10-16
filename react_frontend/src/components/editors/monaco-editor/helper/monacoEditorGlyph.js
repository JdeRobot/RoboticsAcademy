export const monacoEditorGlyph = ({ monaco, editor, setLineNumber }) => {
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
  lineNumberDecorationRef,
  allLineNumberDecorations
) => {
  const allGlyphs = [];
  if (!allLineNumberDecorations) return;

  allLineNumberDecorations.forEach((line, i) => {
    const newDecoration = {
      range: new monaco.Range(line, 1, line, 1),
      options: {
        glyphMarginClassName: "glyph-indicator",
        glyphMarginHoverMessage: {
          value: ``,
        },
      },
    };
    allGlyphs.push(newDecoration);
  });

  // Add the new decoration to the decorations collection
  lineNumberDecorationRef.current.set(allGlyphs);
};

// filter line number
export const filterLineNumber = ({
  lineNumberDecorations,
  lineNumber,
  maxEditorRows,
}) => {
  let allLineNumberDecorations;

  // Filter all no existing rows
  allLineNumberDecorations = lineNumberDecorations.filter(
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
