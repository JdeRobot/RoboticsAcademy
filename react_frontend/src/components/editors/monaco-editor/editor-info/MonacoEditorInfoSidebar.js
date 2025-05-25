import React from "react";
import { KeyboardIcon, WidgetsIcon } from "../icons";

const MonacoEditorInfoSidebar = ({ editorSettings, dispatch }) => {
  const { modalScreenState } = editorSettings;
  return (
    <div className="flex flex-col items-center justify-start pt-10 gap-2 w-[120px] h-full bg-[#383838] rounded-xl  select-none">
      {/* shortcuts */}
      <div
        className={`flex items-center justify-center  gap-1 w-[100px] h-7 rounded-full cursor-pointer ${
          modalScreenState === "shortcuts" ? `bg-[#FFA726]` : ``
        }`}
        onClick={() =>
          dispatch({
            type: "changeModalScreenState",
            payload: { screen: "shortcuts" },
          })
        }
      >
        <KeyboardIcon cssClass="" />
        <span className="text-[#fcfcfc] text-sm">shortcuts</span>
      </div>
      {/* widgets */}
      <div
        className={`flex items-center justify-center  gap-1 w-[100px] h-7 rounded-full cursor-pointer ${
          modalScreenState === "widgets" ? `bg-[#FFA726]` : ``
        }`}
        onClick={() =>
          dispatch({
            type: "changeModalScreenState",
            payload: { screen: "widgets" },
          })
        }
      >
        <WidgetsIcon cssClass="" />
        <span className="text-[#fcfcfc] text-sm">widgets</span>
      </div>
    </div>
  );
};

export default MonacoEditorInfoSidebar;
