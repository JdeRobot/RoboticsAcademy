<a href="https://mmg-ai.com/en/"><img src="https://jderobot.github.io/assets/images/logo.png" width="100 " align="right" /></a>

# Contributing to RoboticsAcademy

First off, thanks for your interest in contributing to RoboticsAcademy! All contributors are welcome, from commenting issues to reviewing or sending Pull Requests.

## How to contribute?

If you are new to GitHub, visit the [first-contributions instructions](https://github.com/firstcontributions/first-contributions/blob/master/README.md) to learn how to contribute on GitHub.

To find issues you can help with, go though the list of [good first issues](https://github.com/JdeRobot/RoboticsAcademy/issues?q=is%3Aissue+is%3Aopen+label%3Agood-first-issue) or issues labeled with [help wanted](https://github.com/JdeRobot/RoboticsAcademy/issues?q=is%3Aissue+is%3Aopen+label%3A%22help+wanted%22).

Once found or created an issue, let us know that you want to work on it by commenting in the issue.

**Important**: this is Robotics Academy and it works as Robotics Academy with its own structure and codebase. If you are not able comprehend it, this may not be a good project to contribute to. Submitting changes (Pull Request) or issues without that or without reading the documentation will be considered as **spam**. An example of bad submissions are: disabling the database, frontend and backend to "simplify development for low-risk developers" in [PR 3411](https://github.com/JdeRobot/RoboticsAcademy/pull/3411), or submitting a standalone python script that tries to fix an issue that is not there in [PR 3421](https://github.com/JdeRobot/RoboticsAcademy/pull/3421).

## Opening a Pull Request

If you have fixed an issue and want to share your fix create a pull request. If your pull request does not follow the next points it will not be accepted:

- Fixes the issue related to the pull request
- Does not contain any additional code than the one related to the fix
- Has been tested and compiled with a corresponding video or image. **Not a link to another webpage, you must add the video or image with Github's add file feature.**
- If the changes are still in progress open a Draft instead of a Pull Request. All PR will be considered as ready to merge
- The changes submited must be up to date with the latest version of the branch they are being submitted to

If it breaks one of the points above you will be requested to change it and if you do not it will be closed.

## Questions, suggestions or new ideas

Please don't open an issue to ask a question or suggestion. Use the [GitHub Discussions](https://github.com/JdeRobot/RoboticsAcademy/discussions) which are meant for that. New ideas and enhacements are also welcome as discussion posts.

## Issue reporting

Feel free to [create a new issue](https://github.com/JdeRobot/RoboticsAcademy/issues/new) if you have some issue to report. But first, make sure that the issue has not been reported yet.

Be sure to explain in details the context and the outcome that you are lookign for. If reporting bugs, provide basic information like you OS version, RoboticsBackend version and the exercise launched.

The responses to the issues **must** be related to the topic and they must not contain any solutions, for that use a Pull Request.

## How to contribute in exercises documentation?

The user documentation for each exercise has to be in GitHub Pages [gh-pages branch at the RoboticsAcademy repository](https://github.com/JdeRobot/RoboticsAcademy/tree/gh-pages), in markdown format. By default, the information added to the documentation is split in two:

- A `.md` file with the description of the exercise, steps for execution, comments, etc.
- A folder with resources. Typically images and videos that are associated to the exercise.

The file with the information is placed in the path: `/_pages/exercises/<section>/<exercise_name>.md` with the name of the exercise and with Markdown extension (`.md`).

The images associated to this file with the information are stored in the path: `/assets/images/exercises/<exercise_name>/`

### Name policy

The name policy is simple:

All file names attached to the documentation will be **lowercase** and spaces will be replaced by **underscore** ("`_`") using ASCII characters.

### Images policy

In order to maintain coherence between all the exercises, it is necessary to distinguish between one image and the rest. This is the image shown in the set of exercises.

The image name policy for teaser is `<exercise_name>_teaser`. (Note how it has to end in `_teaser`). For teaser images the required aspect ratio has to be multiple 9/10.

The rest of the images have no name restriction. When in doubt, they are named as the exercise and with a number behind them.

- For the images of the exercises or menus use the [name of the Minimal Mistakes gallery](https://mmistakes.github.io/minimal-mistakes/post%20formats/post-gallery/).

### Exercise points

All exercises will contain the following points:

1. **Goal**: Brief description of the objective of the exercise
2. **Installation**: If the exercise requires an additional installation, the additional commands are added in this section.
3. **How to perform the exercise?**: The files to be modified to solve the exercise are defined as well as the functions available to obtain information from the sensors and send to the actuators.
4. **How to run your solution?**: The commands and settings to be launched to test the developed code are specified
5. **Theory**: The information about the exercise is described **in detail**.
6. **Hints**: Clues are given as to where the exercise can be tackled.
7. **Demonstrate Video**: Video with progress, clues and/or the result of the exercise
8. **Contributors**: Two groups are described: contributors (all the original authors who developed the exercise code) and the maintainer (the person responsible for the exercise, updating it, correcting errors, resolving doubts, etc, is the contributor who knows the exercise best at present).
9. **References**: Information on the points described in the theory section, articles, websites with information, etc.

_Note: If at any point in the exercise description it is required to add mathematical equations, these must be put into the equation format in LaTeX. You can use this [online equation generator](https://www.hostmath.com/) for ease of use. For more information about equations in LaTeX, check their [documentation](https://en.wikibooks.org/wiki/LaTeX/Mathematics). _

Thanks! :heart: :heart:
RoboticsAcademy Team
