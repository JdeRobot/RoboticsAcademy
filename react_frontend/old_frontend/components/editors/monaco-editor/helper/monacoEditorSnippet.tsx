import { basic_snippets, guiAndHalAutoCompleteObj, frequencyAutoCompleteObj } from "./../index";
import { editor, languages } from "monaco-editor";
import EventEmitter from "events";

interface SnippetItem {
  label: string;
  type: string;
  detail?: string;
  code: string;
  docstring?: string;
}

// Main Editor Snippets
export const monacoEditorSnippet = ({ monaco }: { monaco: typeof import("monaco-editor") }) => {
  monaco.languages.register({ id: "python" });

  const bus = new EventEmitter();
  let lock = true;

  // Register a completion item provider for the new language
  monaco.languages.registerCompletionItemProvider("python", {
    triggerCharacters: [".", "("],
    provideCompletionItems: async (model, position) => {
      lock = true;

      var word = model.getWordUntilPosition(position);
      var prevWord = model.getWordUntilPosition({
        lineNumber: position.lineNumber,
        column: position.column - 1,
      });

      var range: Range = {
        startLineNumber: position.lineNumber,
        endLineNumber: position.lineNumber,
        startColumn: word.startColumn,
        endColumn: word.endColumn,
      };

      // Add basic snippets only if not prevWord
      let snippets = prevWord.word === ""
        ? snippetsBuilderV2("basic_snippets", monaco, range, "")
        : [];

      if (["WebGUI", "HAL", "Frequency"].includes(prevWord.word)) {
        const suggestions = snippetsBuilderV2("hal_gui", monaco, range, prevWord.word);
        return { suggestions };
      }
      
      // Check if the Robotics Backend is connected
      // Call the RAM for autocompletion
      try {
        window.RoboticsExerciseComponents.commsManager.code_autocomplete({
          code: model.getValue(),
          line: position.lineNumber,
          col: word.endColumn - 1,
        });
      } catch (error) {
        return { suggestions: snippets };
      }

      const callback = (message: any) => {
        const data = message.data;

        if (!data) return;

        const new_completions: SnippetItem[] = data.completions;

        new_completions.forEach((snippet) => {
          snippets.push({
            label: snippet.label,
            kind: snippetKind({ kind: snippet.type, monaco }),
            detail: snippet.detail,
            documentation: snippet.docstring,
            insertText: snippet.code,
            insertTextRules:
              monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
            range: range,
          });
        });

        lock = false;
        bus.emit("unlocked");
      };

      window.RoboticsExerciseComponents.commsManager.suscribreOnce(
        [
          window.RoboticsExerciseComponents.commsManager.events
            .CODE_AUTOCOMPLETE,
        ],
        callback
      );

      if (lock) await new Promise((resolve) => bus.once("unlocked", resolve));

      return { suggestions: snippets };
    },
  });
};

// Snippets Builder
export const snippetKind = ({ kind, monaco }: { kind: string; monaco: typeof import("monaco-editor") }) => {
  switch (kind) {
    case "variable":
      return monaco.languages.CompletionItemKind.Variable;
    case "class":
      return monaco.languages.CompletionItemKind.Class;
    case "param":
      return monaco.languages.CompletionItemKind.TypeParameter;
    case "path":
      return monaco.languages.CompletionItemKind.File;
    case "property":
      return monaco.languages.CompletionItemKind.Property;
    case "statement":
      return monaco.languages.CompletionItemKind.Function;
    case "instance":
      return monaco.languages.CompletionItemKind.Class;
    case "module":
      return monaco.languages.CompletionItemKind.Module;
    case "method":
      return monaco.languages.CompletionItemKind.Method;
    case "snippet":
      return monaco.languages.CompletionItemKind.Snippet;
    case "keyword":
      return monaco.languages.CompletionItemKind.Keyword;
    case "function":
      return monaco.languages.CompletionItemKind.Function;
    default:
      return monaco.languages.CompletionItemKind.Variable;
  }
};

// hal & gui auto complete
export const getHalGuiMethods = (importName: string): SnippetItem[] => {
  const pathName = window.location.pathname;
  let exerciseName = pathName.split("/").filter(Boolean);
  exerciseName = exerciseName[exerciseName.length - 1];
  exerciseName = `_${exerciseName}`;

  // if no object found by exercise name
  if (!guiAndHalAutoCompleteObj[exerciseName]) {
    return [];
  }

  if (importName === "WebGUI") {
    return guiAndHalAutoCompleteObj[exerciseName].webgui;
  } else if (importName === "HAL") {
    return guiAndHalAutoCompleteObj[exerciseName].hal;
  } else if (importName === "Frequency") {
    return frequencyAutoCompleteObj;
  }

  return [];
};

export const snippetsBuilderV2 = (
  snippetName: string,
  monaco: typeof import("monaco-editor"),
  range: Range,
  importName: string
): monaco.languages.CompletionItem[] => {
  //const snippets = [];
  let importSnippets: SnippetItem[] = [];

  // basic_snippets
  if (snippetName === "basic_snippets") {
    importSnippets = basic_snippets;
  } else if (snippetName === "hal_gui") {
    // hal_gui
    importSnippets = getHalGuiMethods(importName);
  }

  if (!importSnippets || !importSnippets.length) return [];

  return importSnippets.map((snippet) => ({
    label: snippet.label,
    kind: snippetKind({ kind: snippet.type, monaco }),
    detail: snippet.detail,
    insertText: snippet.code,
    insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
    range: range,
  }));
};

