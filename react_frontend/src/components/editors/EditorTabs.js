import React from "react";
import PropTypes from "prop-types";
import { editorList, monacoEditorThemeList, resizeList } from "./monaco-editor";
import {
  LockIcon,
  MaximizeIcon,
  MinimizeIcon,
  UnlockIcon,
} from "../icons/Icons";

const EditorTabs = ({ state, dispatch }) => {
  const { activeEditor, resizeEditor, monacoEditorTheme } = state;
  const { readOnly } = state.editorOptions;

  return (
    <div className="w-full h-[28px] flex items-center justify-between pr-1 border-b border-[rgba(33,33,0,.5)] bg-white">
      {/* Editor Tabs */}
      <div className="flex items-center justify-start gap-1">
        <div
          className={`${
            activeEditor === editorList[0] && `bg-yellow-500  text-white`
          } px-4 py-1 text-sm font-semibold text-[#333]  hover:cursor-pointer rounded-tl-md rounded-tr-md`}
          onClick={() =>
            dispatch({
              type: "changeEditor",
              payload: { editor: editorList[0] },
            })
          }
        >
          Ace
        </div>
        <div
          className={`${
            activeEditor === editorList[1] && `bg-yellow-500  text-white`
          } px-2 py-1 text-sm font-semibold text-[#333]  hover:cursor-pointer rounded-tl-md rounded-tr-md`}
          onClick={() =>
            dispatch({
              type: "changeEditor",
              payload: { editor: editorList[1] },
            })
          }
        >
          Monaco
        </div>
      </div>
      {/* Editor Options Settings */}
      <div className="flex items-center gap-1">
        {activeEditor === editorList[1] && (
          <>
            {/* Monaco Editor Theme */}
            <select
              className="text-sm text-[#333] bg-slate-300"
              value={monacoEditorTheme}
              onChange={(e) =>
                dispatch({
                  type: "updateMonacoEditorTheme",
                  payload: { theme: e.target.value },
                })
              }
            >
              {monacoEditorThemeList.map((theme, i) => (
                <option value={theme} key={i} className="text-sm text-[#333]">
                  {theme.split("-").join(" ").toLocaleUpperCase()}
                </option>
              ))}
            </select>
            {/*  Lock Editor*/}
            <div
              className="p-2 rounded-full hover:bg-slate-200 cursor-pointer"
              onClick={() => dispatch({ type: "updateLockEditor" })}
            >
              {readOnly ? (
                <LockIcon cssClass="fill-yellow-500" />
              ) : (
                <UnlockIcon cssClass="" />
              )}
            </div>
          </>
        )}{" "}
        {/* Resize Editor */}
        <div
          className="p-2 rounded-full hover:bg-slate-200 cursor-pointer"
          onClick={() => dispatch({ type: "updateEditorResize" })}
        >
          {resizeEditor === resizeList[0] && <MaximizeIcon cssClass="" />}
          {resizeEditor === resizeList[1] && <MinimizeIcon cssClass="" />}
        </div>
      </div>
    </div>
  );
};

export default EditorTabs;

EditorTabs.prototype = {
  state: PropTypes.object.isRequired,
  dispatch: PropTypes.func.isRequired,
};
