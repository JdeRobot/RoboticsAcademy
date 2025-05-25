import React from "react";

import MonacoEditorInfoButtons from "./MonacoEditorInfoButtons";
import MonacoEditorInfoSidebar from "./MonacoEditorInfoSidebar";
import MonacoEditorInfoDetails from "./MonacoEditorInfoDetails";

const MonacoEditorInfo = ({ editorSettings, dispatch, editorRef }) => {
  const { isModalOpen, isCodeFormatEnable, isZoomingEnable, modalScreenState } =
    editorSettings;

  const ctrlSEvent = new KeyboardEvent("keydown", {
    key: "s",
    ctrlKey: true,
    bubbles: true,
  });

  const handleFormatCode = () => {
    editorRef.current.getDomNode().dispatchEvent(ctrlSEvent);
  };

  const handleFontZoom = (zoom) => {
    const currentFontSize = editorRef.current.getOption(
      monaco.editor.EditorOption.fontSize
    );

    // font size between 10 and 100
    if (zoom === "up") {
      editorRef.current.updateOptions({
        fontSize: Math.min(100, currentFontSize + 1),
      });
    } else {
      editorRef.current.updateOptions({
        fontSize: Math.max(10, currentFontSize - 1),
      });
    }
  };

  return (
    <>
      {isModalOpen && (
        <>
          {/* blur layer */}
          <div
            className="w-full h-full absolute left-0 right-0 z-[99] "
            style={{
              background: "rgba(0, 0, 0, 0.02)",
              filter: "blur(2px)",
              backdropFilter: "blur(2px)",
            }}
          ></div>
          {/* modal */}
          <div
            className="flex absolute w-[400px] h-[250px] left-[50%] top-[50%] bg-[#2D2D2D]  rounded-xl -translate-x-[50%] -translate-y-[50%] z-[100]"
            style={{ border: "1px solid #464646" }}
          >
            {/* sizebar */}
            <MonacoEditorInfoSidebar
              dispatch={dispatch}
              editorSettings={editorSettings}
            />
            {/* main */}
            <MonacoEditorInfoDetails
              editorSettings={editorSettings}
              dispatch={dispatch}
            />
          </div>
        </>
      )}
      {/* buttons */}
      <MonacoEditorInfoButtons
        editorSettings={editorSettings}
        dispatch={dispatch}
        handleFontZoom={handleFontZoom}
        handleFormatCode={handleFormatCode}
      />
    </>
  );
};

export default MonacoEditorInfo;

/*



background: rgba(0, 0, 0, 0.002);
filter: blur(3px);
backdrop-filter: blur(3px);


*/
