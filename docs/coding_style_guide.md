This is the coding style guide for languages used in the JdeRobot repositories.

> [!note]
> This guide is heavily based on the [ROS2 (Humble) Coding Style Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)

### Python

-   Use the [PEP8 guidelines](https://www.python.org/dev/peps/pep-0008/) for code formatting
    -   changes:
        -   [Maximum Line Length](https://peps.python.org/pep-0008/#maximum-line-length) = 88
-   Use the [PEP257 guidelines](https://peps.python.org/pep-0257/) for docstrings formatting

---

-   Enforce using
    -   Linter: `flake8`,
    -   Formatter: `black`
    -   Docstrings validation: `pydocstyle`

### C++

-   Use the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) with the modifications stated in the ROS2 Style Guide

---

-   Enforce using `ament_cpplint` and `ament_uncrustify`

### CMAKE

-   Use the style guide defined by the [ROS2 style guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html#cmake) with some extra modifications

**Summary**

-   Max line length: 100 characters
-   Use lowercase command names (`find_package`, not `FIND_PACKAGE`).
-   Use `snake_case` identifiers (variables, functions, macros).
-   Use empty `else()` and `end...()` commands.
-   No whitespace before `(`‘s.
-   Use two spaces of indentation, do not use tabs.
-   Do not use aligned indentation for parameters of multi-line macro invocations. Use two spaces only.
-   Prefer functions with `set(PARENT_SCOPE)` to macros.
-   When using macros prefix local variables with `_` or a reasonable prefix.

---

-   Enforce using `ament_lint_cmake`

### Javascript/Typescript

-   Use the recommended [ESLint](https://eslint.org/docs/latest/use/configure/configuration-files#using-predefined-configurations) configuration as base, with extra plugins for specific frameworks or tools
    -   Example extensions:
        -   `react` for `jsx` components linting
        -   `json` for `JSON` linting

---

-   Enforce using
    -   Linter: `eslint`
    -   Formatter: `prettier`

**Base ESLint Config**

```json
{
    "extends": ["eslint:recommended", "prettier"],
    "env": {
        "browser": true,
        "es2021": true,
        "node": true
    },
    "parserOptions": {
        "ecmaVersion": 12,
        "sourceType": "module"
    },
    "rules": {
        // Add custom rules if needed
    }
}
```

### HTML/CSS

-   Use the [Google HTML/CSS Style Guide](https://google.github.io/styleguide/htmlcssguide.html#CSS)

---

-   Enforce using
    -   Linters: `eslint`, `stylelint`
    -   Formatter: `prettier`

### XML

-   Max line width: 80 characters
-   Use spaces for indentation. No tabs in content.
-   Indentation of 2 spaces per level
-   use double quotes for strings

---

-   Enforce using `xmllint`/`ament_xmllint`

### YAML

-   Use spaces for indentation. No tabs in content.
-   Indentation of 2 spaces per level
-   Use double quotes for all strings
-   check this out: https://developers.home-assistant.io/docs/documenting/yaml-style-guide/

---

-   Enforce using
    -   Linter: `yamllint`
    -   Formatter: `yamlfmt`

# How To Setup Your IDE

## VSCode

### Python

-   Install needed packages in your python environment

```bash
pip install flake8 flake8-docstrings pydocstyle black
```

-   Install the [flake8 linter vscode extension](https://marketplace.visualstudio.com/items?itemName=ms-python.flake8)
-   Install the [black formatter vscode extension](https://marketplace.visualstudio.com/items?itemName=ms-python.black-formatter)
-   Add the following to your `settings.json` file

```json
{
    // ...

    "[python]": {
        "editor.defaultFormatter": "ms-python.black-formatter"
    },
    "black-formatter.args": ["--line-length", "88"],
    "flake8.importStrategy": "fromEnvironment",
    "flake8.args": [
        "--max-line-length=88",
        "--extend-ignore=E701,W503,E203",
        "--docstring-convention=pep257"
    ]

    // ...
}
```

-   Docstrings
    -   consider installing the [autoDocString extension](https://marketplace.visualstudio.com/items?itemName=njpwerner.autodocstring) to auto generate docstrings

### Javascript - HTML - CSS

-   Install the [prettier vscode extension](https://marketplace.visualstudio.com/items?itemName=esbenp.prettier-vscode)
-   Install the [eslint vscode extension](https://marketplace.visualstudio.com/items?itemName=dbaeumer.vscode-eslint)

-   add the following `.prettierrc` file to the source code

```json
{
    "printWidth": 80,
    "tabWidth": 2,
    "useTabs": false,
    "semi": true,
    "singleQuote": true,
    "trailingComma": "es5",
    "bracketSpacing": true,
    "arrowParens": "always",
    "endOfLine": "lf",
    "overrides": []
}
```

-   Install the needed modules for the project languages

```bash
# Common & JS
npm install --save-dev eslint eslint-plugin-prettier eslint-config-prettier

# Typescript
npm install --save-dev typescript typescript-eslint

# React JSX
npm install --save-dev eslint-plugin-react eslint-plugin-react-hooks eslint-plugin-jsx-a11y

# HTML
npm install --save-dev @html-eslint/eslint-plugin @html-eslint/parser

# CSS
npm install --save-dev @eslint/css
```

-   Add the following `.eslintrc.json` to the source code

```json
{
    "root": true,
    "env": {
        "browser": true,
        "node": true,
        "es2021": true
    },
    "parserOptions": {
        "ecmaVersion": 2021,
        "sourceType": "module"
    },
    "extends": ["eslint:recommended", "plugin:prettier/recommended"],
    "plugins": ["prettier", "@html-eslint", "yml"],
    "overrides": [
        // {
        //   "files": ["*.ts", "*.tsx"],
        //   "parser": "@typescript-eslint/parser"
        // },
        {
            "files": ["**/*.html"],
            "parser": "@html-eslint/parser"
        },
        {
            "files": ["*.css", "*.scss"]
        },
        {
            "files": ["*.yml", "*.yaml"],
            "extends": ["plugin:yml/recommended"]
        }
    ],
    "rules": {
        "indent": ["error", 2],
        "quotes": ["error", "single"],
        "semi": ["error", "always"]
        // "@typescript-eslint/no-unused-vars": "warn"
    }
}
```

#### CSS

-   Install the [Stylelint VSCode Extension](https://marketplace.visualstudio.com/items?itemName=stylelint.vscode-stylelint)
-   Install the `stylelint` needed modules

```bash
npm install --save-dev stylelint stylelint-config-standard
```

-   Add the following `.stylelintrc.json` to your source code

```json
{
    "extends": ["stylelint-config-standard"]
}
```

### YAML

-   Install the [Prettier Vscode Extension](https://marketplace.visualstudio.com/items?itemName=esbenp.prettier-vscode)
-   Install the [RedHat YAML Extension](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-yaml)
-   add the following `.prettierrc` file to the projects source code

```json
{
    "printWidth": 80,
    "tabWidth": 2,
    "useTabs": false,
    "semi": true,
    "singleQuote": true,
    "trailingComma": "es5",
    "bracketSpacing": true,
    "arrowParens": "always",
    "endOfLine": "lf",
    "overrides": [
        {
            "files": ["*.yaml", "*.yml"],
            "options": {
                "singleQuote": false
            }
        }
    ]
}
```

-   OR add the following to your vscode `settings.json` file

```json
{
    // ...
    "editor.formatOnSave": true,
    "[yaml]": { "editor.defaultFormatter": "esbenp.prettier-vscode" },
    //
    "prettier.printWidth": 80,
    "prettier.tabWidth": 2,
    "prettier.useTabs": false,
    "prettier.semi": true,
    "prettier.singleQuote": true,
    "prettier.trailingComma": "es5",
    "prettier.bracketSpacing": true,
    "prettier.arrowParens": "always",
    "prettier.endOfLine": "lf",
    //
    "[yaml]": {
        "prettier.singleQuote": false
    }
    // ...
}
```

### XML

-   install the [RedHat XML Extenstion](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-xml)
-   Add the following to your vscode `settings.json` file

```json
{
    //
    "[xml]": {
        "editor.defaultFormatter": "redhat.vscode-xml"
    },
    "xml.format.enabled": true,
    "xml.preferences.quoteStyle": "double",
    "xml.format.enforceQuoteStyle": "preferred",
    "xml.format.maxLineWidth": 80
    //
}
```

### C++

-   Install `cpplint` and `uncrustify` (Ubuntu)

```bash
pip install cpplint
sudo apt install uncrustify
```

-   Install the [cpplint VSCode Extension](https://marketplace.visualstudio.com/items?itemName=mine.cpplint)
-   Add the following to your vscode `settings.json` file
    -   run `which cpplint` to find `<your-cpplint-path>`

```json
{
    //
    "cpplint.cpplintPath": "<your-cpplint-path>",
    "cpplint.lineLength": 100,
    "cpplint.filters": [
        "-build/include_subdir",
        "-whitespace/braces",
        "-build/namespaces",
        "-whitespace/indent",
        "-whitespace/indent_namespace",
        "-whitespace/parens",
        "+runtime/indentation_namespace"
    ],
    "cpplint.extensions": [
        "cpp",
        "h++",
        "cuh",
        "c",
        "c++",
        "cu",
        "hxx",
        "hpp",
        "cc",
        "cxx",
        "h"
    ],
    "cpplint.headers": ["h++", "cuh", "hxx", "hpp", "h"]
    //
}
```

-   Install the [uncrustify VSCode Extension](https://marketplace.visualstudio.com/items?itemName=zachflower.uncrustify)
    -   Add an `uncrustify.cfg` file to your project if not present
        -   content should be the same as the [ROS2 config file](https://github.com/ament/ament_lint/blob/humble/ament_uncrustify/ament_uncrustify/configuration/ament_code_style.cfg)

### CMAKE

-   Install needed packages

```bash
pip install cmakelint cmake-format
```

-   Install the [cmake-format VSCode Extension](https://marketplace.visualstudio.com/items?itemName=cheshirekow.cmake-format)
-   Add the following to your vscode `settings.json`

```json
{
    //
    "cmakeFormat.args": [
        "--line-width=80",
        "--tab-size=2",
        "--use-tabchars=false",
        "--command-case=lower",
        "--separate-fn-name-with-space=false",
        "--separate-ctrl-name-with-space=false",
        "--dangle-parens=false",
        "--max-prefix-chars=2",
        "--max-lines-hwrap=0",
        "--disable=autosort",
        "--enable-markup=true"
    ]
    //
}
```

### Full VSCode Settings

```json
{
    // Global: format on save
    "editor.formatOnSave": true,
    // ─── Python ─────────────────────────────────────────────────────────────
    "[python]": {
        "editor.defaultFormatter": "ms-python.black-formatter"
    },
    "black-formatter.args": ["--line-length", "88"],
    "flake8.importStrategy": "fromEnvironment",
    "flake8.args": [
        "--max-line-length=88",
        "--extend-ignore=E701,W503,E203",
        "--docstring-convention=pep257"
    ],
    // ─── JavaScript / Typescript / HTML / CSS / YAML ────────────────────────────────────────────
    "[javascript]": {
        "editor.defaultFormatter": "esbenp.prettier-vscode"
    },
    "[typescript]": {
        "editor.defaultFormatter": "esbenp.prettier-vscode"
    },
    "[javascriptreact]": {
        "editor.defaultFormatter": "esbenp.prettier-vscode"
    },
    "[typescriptreact]": {
        "editor.defaultFormatter": "esbenp.prettier-vscode"
    },
    "[html]": {
        "editor.defaultFormatter": "esbenp.prettier-vscode"
    },
    "[css]": {
        "editor.defaultFormatter": "esbenp.prettier-vscode"
    },
    "[yaml]": {
        "editor.defaultFormatter": "esbenp.prettier-vscode",
        "prettier.singleQuote": true
    },
    "eslint.enable": true,
    "eslint.run": "onType",
    "eslint.validate": [
        "javascript",
        "typescript",
        "javascriptreact",
        "typescriptreact",
        "html"
    ],
    "css.validate": false,
    "scss.validate": false,
    "less.validate": false,
    // ─── C++ ────────────────────────────────────────────────────────────────
    "cpplint.cpplintPath": "/home/abdallah/.local/bin/cpplint",
    "cpplint.lineLength": 100,
    "cpplint.filters": [
        "-build/include_subdir",
        "-whitespace/braces",
        "-build/namespaces",
        "-whitespace/indent",
        "-whitespace/indent_namespace",
        "-whitespace/parens",
        "+runtime/indentation_namespace"
    ],
    "cpplint.extensions": [
        "cpp",
        "h++",
        "cuh",
        "c",
        "c++",
        "cu",
        "hxx",
        "hpp",
        "cc",
        "cxx",
        "h"
    ],
    "cpplint.headers": ["h++", "cuh", "hxx", "hpp", "h"],
    // ─── CMake ─────────────────────────────────────────────────────────────
    "cmakeFormat.args": [
        "--line-width=80",
        "--tab-size=2",
        "--use-tabchars=false",
        "--command-case=lower",
        "--separate-fn-name-with-space=false",
        "--separate-ctrl-name-with-space=false",
        "--dangle-parens=false",
        "--max-prefix-chars=2",
        "--max-lines-hwrap=0",
        "--disable=autosort",
        "--enable-markup=true"
    ],
    // ─── XML ───────────────────────────────────────────────────────────────
    "[xml]": {
        "editor.defaultFormatter": "redhat.vscode-xml"
    },
    "xml.format.enabled": true,
    "xml.preferences.quoteStyle": "double",
    "xml.format.enforceQuoteStyle": "preferred",
    "xml.format.maxLineWidth": 80
}
```
