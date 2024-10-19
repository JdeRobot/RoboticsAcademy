import { getAllSnippets } from "./../index";

// Main Editor Snippets
export const monacoEditorSnippet = ({
  monaco,
  guiAutoComplete,
  halAutoComplete,
}) => {
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

      //* Class & Objects
      const text = model.getValue();
      const classes = extractClassesAndMembers(text);

      // Match the instance name before the dot (e.g. obj.)
      const classObjMatch = textUntilPosition.match(/(\w+)\.$/);

      //* HAL & GUI autocomplete
      const halGuiMatch = textUntilPosition.match(/(GUI|HAL)\.$/);

      if (halGuiMatch) {
        // HAL & GUI
        const instanceName = halGuiMatch[1]; // either GUI or HAL

        // Define suggestions for GUI and HAL
        const suggestions =
          instanceName === "GUI"
            ? guiAutoComplete
            : instanceName === "HAL"
            ? halAutoComplete
            : [];

        return { suggestions };
      } else if (classObjMatch) {
        // class obj
        const instanceName = classObjMatch[1];
        const className = findClassNameByInstance(text, instanceName);

        if (className && classes[className]) {
          const classInfo = classes[className];
          const suggestions = [
            ...classInfo.attributes.map((attr) => ({
              label: attr,
              kind: monaco.languages.CompletionItemKind.Field,
              insertText: attr,
              documentation: `Attribute of ${className}`,
            })),
            ...classInfo.methods.map((method) => ({
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
