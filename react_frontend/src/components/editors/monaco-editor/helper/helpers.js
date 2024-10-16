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

// post and response code format
export const fetchAnalysisCode = async ({
  baseUrl,
  monacoEditorSourceCode,
  controller,
}) => {
  try {
    const response = await fetch(`${baseUrl}/api/v1/analysis/`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        code: monacoEditorSourceCode,
        disable_errors: [
          ...pylint_error,
          ...pylint_warning,
          ...pylint_convention,
          ...pylint_refactor,
          ...pylint_fatal,
        ],
      }),
      signal: controller.signal,
    });

    const data = await response.json();

    if (response.ok) {
      console.log("data ",data);
      
      return data;
    } else {
      console.error("Error formatting code:", data.error);
    }
  } catch (error) {
    if (error.name !== "AbortError") {
      console.log(error);
    }
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
