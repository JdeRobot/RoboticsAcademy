export const monacoEditorScroll = ({ editor }) => {
  const domNode = editor.getDomNode();
  domNode.addEventListener("wheel", (event) => {
    const currentFontSize = editor.getOption(
      monaco.editor.EditorOption.fontSize
    );

    // font size between 10 and 100
    if (event.ctrlKey) {
      event.preventDefault();

      if (event.deltaY < 0) {
        editor.updateOptions({
          fontSize: Math.min(100, currentFontSize + 1),
        });
      } else {
        editor.updateOptions({
          fontSize: Math.max(10, currentFontSize - 1),
        });
      }
    }
  });
};
