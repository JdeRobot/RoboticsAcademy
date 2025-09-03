import * as monaco from "monaco-editor";

// Types
type Range = monaco.IRange;

type CompletionItem = monaco.languages.CompletionItem;

// Extract Variables
export const getEditorVariables = ({ lines, monaco, range }: {
  lines: string[];
  monaco: typeof import("monaco-editor");
  range: Range;
}): CompletionItem[] => {
  const variablesSet = new Set<string>();

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
export const getEditorFunctions = ({ lines, monaco, range }: {
  lines: string[];
  monaco: typeof import("monaco-editor");
  range: Range;
}): CompletionItem[] => {
  const functionsSet = new Set<string>();
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

export type ClassStructure = {
  [className: string]: {
    attributes: string[];
    methods: string[];
    spaceSize: number;
  };
};

// Class Object
export const extractClassesAndMembers = (code: string): ClassStructure => {
  const classPattern = /class (\w+)\s*:/g;
  const methodPattern = /def (\w+)\(/g;
  const attributePattern = /(\w+)\s*=/;

  const classes: ClassStructure = {};
  let currentClass: string | null = null;

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
export const findClassNameByInstance = (
  code: string,
  instanceName: string
): string | null => {
  const instancePattern = new RegExp(`${instanceName}\\s*=\\s*(\\w+)\\(`);
  const match = instancePattern.exec(code);
  return match ? match[1] : null;
};

// import data extractor
export type PythonImport = {
  importName: string;
  alias: string;
};

export const extractPythonImports = (code: string): PythonImport[] => {
  const importRegex =
    /\bimport\s+([a-zA-Z_][\w]*)(?:\s+as\s+([a-zA-Z_][\w]*))?/g;

  const imports: PythonImport[] = [];
  let match: RegExpExecArray | null;

  while ((match = importRegex.exec(code)) !== null) {
    const importName = match[1];
    const alias = match[2] ? match[2] : importName;
    imports.push({ importName, alias });
  }

  return imports;
};
