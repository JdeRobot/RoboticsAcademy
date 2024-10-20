import {
  pylint_error,
  pylint_warning,
  pylint_convention,
  pylint_refactor,
  pylint_fatal,
  guiAndHalAutoCompleteObj,
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

// hal & gui auto complete
export const getHalGuiMethods = ({ monaco }) => {
  const pathName = window.location.pathname;
  let exerciseName = pathName.split("/").filter(Boolean);
  exerciseName = exerciseName[exerciseName.length - 1];
  exerciseName = `_${exerciseName}`;

  // if no object found by exercise name
  if (!guiAndHalAutoCompleteObj[exerciseName]) {
    return { guiAutoComplete: [], halAutoComplete: [] };
  }

  const guiAutoComplete = guiAndHalAutoCompleteObj[exerciseName].gui.map(
    (g, i) => {
      return {
        label: g.label,
        kind:
          g.type === "method"
            ? monaco.languages.CompletionItemKind.Method
            : monaco.languages.CompletionItemKind.Variable,
        insertText: g.code,
        documentation: g.descriptions,
      };
    }
  );

  const halAutoComplete = guiAndHalAutoCompleteObj[exerciseName].hal.map(
    (h, i) => {
      return {
        label: h.label,
        kind:
          h.type === "method"
            ? monaco.languages.CompletionItemKind.Method
            : monaco.languages.CompletionItemKind.Variable,
        insertText: h.code,
        documentation: h.descriptions,
      };
    }
  );

  return { guiAutoComplete, halAutoComplete };
};
