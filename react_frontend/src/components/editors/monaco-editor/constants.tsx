export const resizeList = ["min", "max"] as const;
export type ResizeOption = typeof resizeList[number]; 

export const monacoEditorThemeList = ["vs-dark"] as const;
export type MonacoEditorTheme = typeof monacoEditorThemeList[number];

export const defaultEditorSourceCode = `import WebGUI
import HAL
import Frequency
# Enter sequential code!

while True:
    # Enter iterative code!
    Frequency.tick()
` as const;

// Know about pylint Errors with type
// https://gist.github.com/codezerro/f7f696702ee7ea12782d9af3f9bb4f4c
// pylint disable error
export const pylint_error = ["E0401", "E1101"] as const;
export type PylintError = typeof pylint_error[number];

export const pylint_warning = ["W0611"] as const;
export type PylintWarning = typeof pylint_warning[number];

export const pylint_convention = ["C0114", "C0303", "C0304", "C0305", "C0411"] as const;
export type PylintConvention = typeof pylint_convention[number];

export const pylint_refactor = [] as const;
export type PylintRefactor = never;

export const pylint_fatal = [] as const;
export type PylintFatal = never;
