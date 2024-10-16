export function createDependencyProposals(range) {
  // returning a static list of proposals, not even looking at the prefix (filtering is done by the Monaco editor),
  // here you could do a server side lookup
  return [
    {
      label: "for",
      kind: monaco.languages.CompletionItemKind.Snippet,
      documentation: "Snippet for a for loop",
      insertText: "for ${1:item} in ${2:iterable}:\n\t$0",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
    },
    {
      label: "while",
      kind: monaco.languages.CompletionItemKind.Snippet,
      documentation: "Snippet for a while loop",
      insertText: "while ${1:condition}:\n\t$0",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
    },
    {
      label: "if",
      kind: monaco.languages.CompletionItemKind.Snippet,
      documentation: "Snippet for an if statement",
      insertText: "if ${1:condition}:\n\t$0",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
    },
    {
      label: "elif",
      kind: monaco.languages.CompletionItemKind.Snippet,
      documentation: "Snippet for an elif statement",
      insertText: "elif ${1:condition}:\n\t$0",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
    },
    {
      label: "print",
      kind: monaco.languages.CompletionItemKind.Snippet,
      documentation: "Snippet for a print statement",
      insertText: "print(${1:message})",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
    },
    {
      label: "def",
      kind: monaco.languages.CompletionItemKind.Snippet,
      documentation: "Snippet for a function definition",
      insertText: "def ${1:function_name}(${2:params}):\n\t$0",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
    },
    {
      label: "try",
      kind: monaco.languages.CompletionItemKind.Snippet,
      documentation: "Snippet for try-except block",
      insertText: "try:\n\t$0\nexcept ${1:Exception} as ${2:e}:\n\tpass",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
    },
    {
      label: "class",
      kind: monaco.languages.CompletionItemKind.Snippet,
      documentation: "Snippet for a class definition",
      insertText:
        "class ${1:ClassName}(${2:object}):\n\tdef __init__(self, ${3:args}):\n\t\t$0",
      insertTextRules:
        monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      range: range,
    }
  ];
}
