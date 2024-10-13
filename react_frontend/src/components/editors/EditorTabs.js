import React from "react";
import PropTypes from "prop-types";
import { editorList } from "./monaco-editor";

const EditorTabs = ({ state, dispatch }) => {
  const { activeEditor } = state;

  return (
    <div className="w-full h-[24px] bg-red-500 flex items-center justify-between px-4">
      {/* Editor Tabs */}
      <div className="flex items-center justify-start gap-2">
        <div>Ace</div>
        <div>Monaco</div>
      </div>
      {/* Editor Options Settings */}
      <div>x</div>
    </div>
  );
};

export default EditorTabs;

EditorTabs.prototype = {
  state: PropTypes.object.isRequired,
  dispatch: PropTypes.func.isRequired,
};
