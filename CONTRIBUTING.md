<a href="https://mmg-ai.com/en/"><img src="https://jderobot.github.io/assets/images/logo.png" width="100 " align="right" /></a>

# Contributing to RoboticsAcademy

First off, thanks for your interest in contributing to RoboticsAcademy! All contributors are welcome, from commenting issues to reviewing or sending Pull Requests.

## How to contribute?

If you are new to GitHub, visit the [first-contributions instructions](https://github.com/firstcontributions/first-contributions/blob/master/README.md) to learn how to contribute on GitHub.

To find issues you can help with, go though the list of [good first issues](https://github.com/JdeRobot/RoboticsAcademy/issues?q=is%3Aissue+is%3Aopen+label%3Agood-first-issue) or issues labeled with [help wanted](https://github.com/JdeRobot/RoboticsAcademy/issues?q=is%3Aissue+is%3Aopen+label%3A%22help+wanted%22).

Once found or created an issue, let us know that you want to work on it by commenting in the issue.

## Opening a Pull Request

If you have fixed an issue and want to share your fix create a pull request. If your pull request does not follow the next points it will not be accepted:

* Fixes the issue related to the pull request
* Does not contain any additional code than the one related to the fix
* Has been tested and compiled with a corresponding video or image. **Not a link to another webpage, you must add the video or image with Github's add file feature.**

If it breaks one of the points above you will be requested to change it and if you do not it will be closed.

## Questions, suggestions or new ideas

Please don't open an issue to ask a question or suggestion. Use the [GitHub Discussions](https://github.com/JdeRobot/RoboticsAcademy/discussions) which are meant to it. New ideas and enhacements are also welcome as discussion posts.

## Issue reporting

Feel free to [create a new issue](https://github.com/sayanarijit/xplr/issues/new) if you have some issue to report. But first, make sure that the issue has not been reported yet.

Be sure to explain in details the context and the outcome that you are lookign for. If reporting bugs, provide basic information like you OS version, RoboticsBackend version and the exercise launched.

## How to contribute in exercises documentation?

Take a look at our [documentation guide lines](https://jderobot.github.io/RoboticsAcademy/developer_guide) to contribute in github pages related issues.

Thanks! :heart: :heart:
RoboticsAcademy Team


## Developer setup

For contributors who want to run tests locally and develop the project, a small
helper script is provided to create a Python virtual environment and install a
minimal set of test/dev requirements.

Usage:

```bash
chmod +x scripts/setup_dev.sh
./scripts/setup_dev.sh
source .venv/bin/activate
pytest -q
```

The script is intentionally conservative: it installs only packages listed in
`tests/requirements.txt`. For a full runtime install (may require system
packages) see `docs/requirements.txt`.
