import { frequencyAutoCompleteObj, guiAndHalAutoCompleteObj } from "Constants/hal_gui_snippets";
import { Snippet } from "jderobot-ide-interface";

export const getHalGuiMethods = (prevWord: string): Snippet[] => {
  const pathName = window.location.pathname;
  const exerciseNameArray = pathName.split("/").filter(Boolean);
  const exerciseNameRaw = exerciseNameArray[exerciseNameArray.length - 1];
  const exerciseName = `_${exerciseNameRaw}`;

  // if no object found by exercise name
  if (!guiAndHalAutoCompleteObj[exerciseName]) {
    return [];
  }

  if (prevWord === "WebGUI") {
    return guiAndHalAutoCompleteObj[exerciseName].webgui;
  } else if (prevWord === "HAL") {
    return guiAndHalAutoCompleteObj[exerciseName].hal;
  } else if (prevWord === "Frequency") {
    return frequencyAutoCompleteObj;
  }

  return [];
};