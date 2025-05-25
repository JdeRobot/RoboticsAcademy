import {
  pylint_error,
  pylint_warning,
  pylint_convention,
  pylint_refactor,
  pylint_fatal,
} from "../constants";

// post and response code format
export const fetchFormatCode = async ({
  baseUrl,
  monacoEditorSourceCode,
  setMonacoEditorSourceCode,
}) => {
  try {
    const response = await fetch(`${baseUrl}/api/v1/format/`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "X-CSRFToken": context.csrf,
      },

      body: JSON.stringify({
        code: monacoEditorSourceCode,
      }),
    });

    const data = await response.json();
    if (response.ok) {
      setMonacoEditorSourceCode(data.formatted_code);
    } else {
      console.error("Error formatting code:", data.error);
    }
  } catch (error) {
    console.log("error ", error);
  }
};

export const getMarkerSeverity = ({ type, monaco }) => {
  switch (type) {
    case "refactor":
    case "convention":
      return monaco.MarkerSeverity.Info;
    case "error":
      return monaco.MarkerSeverity.Error;
    case "warning":
    case "fatal":
      return monaco.MarkerSeverity.Warning;
    default:
      return monaco.MarkerSeverity.Error;
  }
};

// local storage data
export const setEditorSettingsWidgetsData = (data) => {
  const data_string = JSON.stringify(data);
  localStorage.setItem("editorSettingsWidgets", data_string);
};

export const getEditorSettingsWidgetsData = () => {
  const data = localStorage.getItem("editorSettingsWidgets");

  if (data) return JSON.parse(data);
  return null;
};
