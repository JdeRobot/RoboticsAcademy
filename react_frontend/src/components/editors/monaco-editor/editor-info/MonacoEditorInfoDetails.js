import React from "react";
import { CloseIcon } from "../icons";

const shortcutsDetails = [
  { title: "Code Format", keys: ["ctrl", "s"] },
  { title: "Font Size", keys: ["ctrl", "wheel"] },
];

const widgetsDetails = [
  { title: "Font Size", id: "isZoomingEnable" },
  { title: "Code Format", id: "isCodeFormatEnable" },
];

const MonacoEditorInfoDetails = ({ editorSettings, dispatch }) => {
  const { isModalOpen, modalScreenState, isCodeFormatEnable, isZoomingEnable } =
    editorSettings;

  const hangleChangeWidgets = (e) => {
    const value = e.target.value;
    dispatch({ type: value });
  };
  return (
    <div className="flex  flex-col w-[calc(400px-120px)] h-full select-none">
      {/* close button */}
      <div
        className="absolute top-1 right-1 flex justify-center items-center w-7 h-7 hover:bg-[#393939] rounded-full duration-100 cursor-pointer"
        onClick={() =>
          dispatch({
            type: "changeSettingsModalState",
            payload: { isModalOpen: !isModalOpen },
          })
        }
      >
        <CloseIcon cssClass="" />
      </div>
      {/* BODY */}
      <div className={`w-full h-full pt-10 items-center px-6`}>
        {/* shortcuts */}
        {modalScreenState === "shortcuts" && (
          <div className="w-full flex flex-col items-start gap-5">
            {/*  */}
            {shortcutsDetails.map((shortcut, i) => (
              <div
                className="w-full flex items-start justify-between text-sm text-[#fcfcfc]"
                key={i}
              >
                <div className="">{shortcut.title}</div>
                <div className="flex items-center gap-1">
                  <span className="bg-[#383838] px-2 xpy-[1px] rounded-md border-[1px] border-[#636363]">
                    {shortcut.keys[0]}
                  </span>
                  +
                  <span className="bg-[#383838] px-2 xpy-[1px] rounded-md border-[1px] border-[#636363]">
                    {shortcut.keys[1]}
                  </span>
                </div>
              </div>
            ))}
          </div>
        )}
        {/* widgets */}
        {modalScreenState === "widgets" && (
          <div className="w-full flex flex-col justify-between  gap-5">
            {widgetsDetails.map((widget, i) => (
              <div
                className="w-full flex items-start justify-between text-sm text-[#fcfcfc]"
                key={i}
              >
                <div className="">{widget.title}</div>
                <label className="inline-flex items-center cursor-pointer">
                  <input
                    type="checkbox"
                    value={
                      widget.id === "isCodeFormatEnable"
                        ? `isCodeFormatEnable`
                        : `isZoomingEnable`
                    }
                    className="sr-only peer"
                    checked={
                      widget.id === "isCodeFormatEnable"
                        ? isCodeFormatEnable
                        : isZoomingEnable
                    }
                    onChange={(e) => hangleChangeWidgets(e)}
                  />
                  <div className="relative w-9 h-5 bg-[#383838] rounded-full peer peer-focus:ring-1 peer-focus:ring-[#636363] peer-checked:after:translate-x-full rtl:peer-checked:after:-translate-x-full peer-checked:after:border-white after:content-[''] after:absolute after:top-0.5 after:start-[2px] after:bg-white after:border-gray-300 after:border after:rounded-full after:h-4 after:w-4 after:transition-all dark:border-gray-600 peer-checked:bg-[#FFA726]"></div>
                </label>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
};

export default MonacoEditorInfoDetails;
