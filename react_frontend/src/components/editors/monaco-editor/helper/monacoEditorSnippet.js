import {
  getAllSnippets,
  getHalGuiMethods,
  listed_python_packages,
  snippetsBuilder,
} from "./../index";
import { extractPythonImports } from "./helpers";

// Main Editor Snippets
export const monacoEditorSnippet = ({ monaco }) => {
  monaco.languages.register({ id: "python" });
  // Register a completion item provider for the new language
  monaco.languages.registerCompletionItemProvider("python", {
    triggerCharacters: ["."],
    provideCompletionItems: async (model, position) => {
      var word = model.getWordUntilPosition(position);
      var range = {
        startLineNumber: position.lineNumber,
        endLineNumber: position.lineNumber,
        startColumn: word.startColumn,
        endColumn: word.endColumn,
      };

      // get text until position
      const textUntilPosition = model.getValueInRange({
        startLineNumber: 1,
        startColumn: 1,
        endLineNumber: position.lineNumber,
        endColumn: position.column,
      });
      // get all text data in editor
      const text = model.getValue();

      //! test import start
      const allLines = text.split("\n").filter(Boolean);
      const allImports = [];
      allLines.forEach((line) => {
        const importData = extractPythonImports(line);

        if (importData.length) {
          if (listed_python_packages.includes(importData[0].importName)) {
            allImports.push(importData[0]);
          }
        }
      });

      // check
      if (allImports.length) {
        const importMatch = allImports.find((imp) => {
          const match = textUntilPosition.match(
            new RegExp(`(${imp.alias})\\.$`)
          );
          if (match) return imp;
        });

        if (importMatch) {
          const { importName, alias } = importMatch;
          if (importName === "GUI" || importName === "HAL") {
            const { guiAutoComplete, halAutoComplete } = getHalGuiMethods({
              monaco,
              importName,
            });

            const suggestions =
              importName === "GUI"
                ? guiAutoComplete
                : importName === "HAL"
                ? halAutoComplete
                : [];

            return { suggestions };
          } else {
            const suggestions = snippetsBuilder({
              monaco,
              range,
              importName,
            });
            return { suggestions };
          }
        }
      }
      //! test import end

      //* Class & Objects
      const classes = extractClassesAndMembers(text);
      // Match the instance name before the dot (e.g. obj.)
      const classObjMatch = textUntilPosition.match(/(\w+)\.$/);

      if (classObjMatch) {
        // class obj
        const instanceName = classObjMatch[1];
        const className = findClassNameByInstance(text, instanceName);

        if (className && classes[className]) {
          const attributesSet = [...new Set(classes[className].attributes)];
          const methodsSet = [...new Set(classes[className].methods)];

          const suggestions = [
            ...attributesSet.map((attr) => ({
              label: attr,
              kind: monaco.languages.CompletionItemKind.Field,
              insertText: attr,
              documentation: `Attribute of ${className}`,
            })),
            ...methodsSet.map((method) => ({
              label: method,
              kind: monaco.languages.CompletionItemKind.Method,
              insertText: `${method}()`,
              documentation: `Method of ${className}`,
            })),
          ];

          return { suggestions: suggestions };
        }
      } else {
        // custom suggestions

        // import
        const importSet = new Set();
        allImports.forEach((imp) => {
          // importSet.add(imp.importName);
          importSet.add(imp.alias);
        });
        // Class
        const classesSet = new Set();
        Object.keys(classes).forEach((key) => classesSet.add(key));

        // custom suggestions
        const lines = model.getLinesContent();
        // func
        const functions = getEditorFunctions({ lines, monaco, range });

        // variable
        const variables = getEditorVariables({ lines, monaco, range });

        // Get all pre-defined snippets
        const preDefinedSnippets = getAllSnippets({ monaco, range });

        let suggestions = [
          // import snippet
          // add import
          ...Array.from(importSet).map((imp) => ({
            label: imp,
            kind: monaco.languages.CompletionItemKind.Class,
            insertText: imp,
            range: range,
          })),
          // class-obj
          ...Array.from(classesSet).map((cls) => ({
            label: cls,
            kind: monaco.languages.CompletionItemKind.Class,
            insertText: cls,
            range: range,
          })),
          ...functions,
          ...variables,
          ...preDefinedSnippets,
        ];

        return { suggestions: suggestions };
      }
    },
  });
};

// Extract Variables
const getEditorVariables = ({ lines, monaco, range }) => {
  const variablesSet = new Set();

  lines.forEach((line) => {
    const matches = line.match(/(\w+)\s*=/);
    if (matches) {
      variablesSet.add(matches[1]);
    }
  });

  return Array.from(variablesSet).map((variable) => ({
    label: variable,
    kind: monaco.languages.CompletionItemKind.Variable,
    insertText: variable,
    range: range,
  }));
};

// Extract functions
const getEditorFunctions = ({ lines, monaco, range }) => {
  const functionsSet = new Set();
  lines.forEach((line) => {
    const matches = line.match(/def\s+(\w+)\s*\(/);
    if (matches) {
      functionsSet.add(matches[1]);
    }
  });

  return Array.from(functionsSet).map((func) => ({
    label: func,
    kind: monaco.languages.CompletionItemKind.Function,
    insertText: func,
    range: range,
  }));
};

// Class Object
const extractClassesAndMembers = (code) => {
  const classPattern = /class (\w+)\s*:/g;
  const methodPattern = /def (\w+)\(/g;
  const attributePattern = /(\w+)\s*=/;

  const classes = {};
  let currentClass = null;

  const lines = code.split("\n");

  lines.forEach((line) => {
    let classMatch = classPattern.exec(line);
    let left_space = line.match(/^\s*/)[0].length;

    if (classMatch) {
      currentClass = classMatch[1];
      classes[currentClass] = {
        attributes: [],
        methods: [],
        spaceSize: left_space,
      };
    } else if (currentClass) {
      let methodMatch = methodPattern.exec(line);
      let spaceSize = classes[currentClass].spaceSize;

      // if line has more left space and current class
      if (spaceSize < left_space) {
        // method match
        if (methodMatch) {
          classes[currentClass].methods.push(methodMatch[1]);
        } else {
          // variable match

          // trim the variable
          let tmp_line = line;
          tmp_line.trim();
          const equalIndex = tmp_line.indexOf("=");

          // Extract the substring from the start to the equal sign
          tmp_line =
            equalIndex !== -1
              ? tmp_line.substring(0, equalIndex + 1).trim()
              : null;

          if (tmp_line === null) return;
          // check valid regex variable
          const regex = /^[a-zA-Z_][a-zA-Z0-9_]*\s*=$/;
          const testLine = regex.test(tmp_line);
          if (!testLine) return;

          // exec
          let attributeMatch = attributePattern.exec(tmp_line);

          if (attributeMatch) {
            classes[currentClass].attributes.push(attributeMatch[1].trim());
          }
        }
      }
    }
  });
  return classes;
};
const findClassNameByInstance = (code, instanceName) => {
  const instancePattern = new RegExp(`${instanceName}\\s*=\\s*(\\w+)\\(`);
  const match = instancePattern.exec(code);
  return match ? match[1] : null;
};
