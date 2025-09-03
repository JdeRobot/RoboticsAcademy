import React from "react";
import { AlignIcon, InfoIcon, ZoomInIcon, ZoomOutIcon } from "../icons";

type EditorSettings = {
  isModalOpen: boolean;
  isCodeFormatEnable: boolean;
  isZoomingEnable: boolean;
};

type Props = {
  editorSettings: EditorSettings;
  dispatch: React.Dispatch<{
    type: string;
    payload: Partial<EditorSettings>;
  }>;
  handleFontZoom: (direction: "up" | "down") => void;
  handleFormatCode: () => void;
};

const MonacoEditorInfoButtons: React.FC<Props> = ({
  editorSettings,
  dispatch,
  handleFontZoom,
  handleFormatCode,
}) => {
  const { isModalOpen, isCodeFormatEnable, isZoomingEnable } = editorSettings;
  return (
    <div
      className={`absolute bottom-3 right-3 flex justify-between items-center gap-1 h-9 bg-[#2D2D2D] rounded-full border-[#464646] border-[1px] ${
        isCodeFormatEnable || isZoomingEnable ? `px-[4px]` : `px-[2px] w-[36px]`
      } duration-300  select-none`}
      style={{zIndex: "10"}}
    >
      {/* font size */}
      {isZoomingEnable && (
        <div className="flex items-center gap-1">
          <div className="flex justify-center items-center  h-7 rounded-full cursor-pointer gap-0.5">
            {/* zoom in */}
            <div
              className="flex justify-center items-center  w-[30px] h-7 bg-[#474747] rounded-tl-full rounded-bl-full duration-100 "
              onClick={() => handleFontZoom("up")}
              title="Zoom In"
            >
              <ZoomInIcon cssClass="" />
            </div>
            {/* zoom out */}
            <div
              className="flex justify-center items-center  w-[30px] h-7 bg-[#474747] rounded-tr-full rounded-br-full duration-100 "
              onClick={() => handleFontZoom("down")}
              title="Zoom Out"
            >
              <ZoomOutIcon cssClass="" />
            </div>
          </div>
          <div className="h-6 w-[2px] bg-[#464646]"></div>
        </div>
      )}
      {/* code format icon */}
      {isCodeFormatEnable && (
        <div className="flex items-center gap-1">
          <div
            className="flex justify-center items-center w-7 h-7 rounded-full hover:bg-[#474747] duration-100 cursor-pointer"
            onClick={() => handleFormatCode()}
            title="Format Code"
          >
            <AlignIcon cssClass="" />
          </div>

          <div className="h-6 w-[2px] bg-[#464646]"></div>
        </div>
      )}
      {/* question mark */}
      <div
        className="flex justify-center items-center w-7 h-7 rounded-full hover:bg-[#474747] duration-100 cursor-pointer"
        onClick={() =>
          dispatch({
            type: "changeSettingsModalState",
            payload: { isModalOpen: !isModalOpen },
          })
        }
        title="More Info"
      >
        <InfoIcon cssClass="" />
      </div>
    </div>
  );
};

export default MonacoEditorInfoButtons;
