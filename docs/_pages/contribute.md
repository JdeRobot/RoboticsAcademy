---
permalink: /contribute/
title: "# How to contribute to Robotics Academy?"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC installation"
toc_icon: "cog"
---

This section describes the steps to include an exercise in the Robotics-Academy documentation.

The objective of having documentation in this format is to simplify the developer having to add information from their software, speeding up the process and showing results that fit the structure of Robotics-Academy.


## Where do I add the documentation for a new exercise?

By default, the information added to the documentation is split in two:

- A `.md` file with the description of the exercise, steps for execution, comments, etc.
- A folder with resources. Typically images and videos that are associated to the exercise.

The file with the information is placed in the path: `/_pages/exercises/<section>/<exercise_name>/files.md` with the name of the exercise and with Markdown extension (`.md`) inside a folder with the same name.

The images associated to this file with the information are stored in the path: `/assets/images/exercises/<exercise_name>/`

## Name policy

The name policy is simple:

All file names attached to the documentation will be **lowercase** and spaces will be replaced by **underscore** ("`_`") using ASCII characters.

## Images policy

In order to maintain coherence between all the exercises, it is necessary to distinguish between one image and the rest. This is the image shown in the set of exercises.

The image name policy for teaser is `<exercise_name>_teaser`. (Note how it has to end in `_teaser`). For teaser images the required size has to be multiple $600x400$ px.

The rest of the images have no name restriction. When in doubt, they are named as the exercise and with a number behind them.

- For the images of the exercises or menus use the [name of the Minimal Mistakes gallery](https://mmistakes.github.io/minimal-mistakes/post%20formats/post-gallery/).


## Exercise points

All exercises will contain the following points:

1. **Goal**: Brief description of the objective of the exercise
2. **Installation**: If the exercise requires an additional installation, the additional commands are added in this section.
3. **How to perform the exercise?**: The files to be modified to solve the exercise are defined as well as the functions available to obtain information from the sensors and send to the actuators.
4. **How to run your solution?**: The commands and settings to be launched to test the developed code are specified
5. **Theory**: The information about the exercise is described **in detail**. 
6. **Hints**: Clues are given as to where the exercise can be tackled.
7. **Demonstrate Video**:  Video with progress, clues and/or the result of the exercise
8. **Contributors**: Two groups are described: contributors (all the original authors who developed the exercise code) and the maintainer (the person responsible for the exercise, updating it, correcting errors, resolving doubts, etc, is the contributor who knows the exercise best at present). 
9. **References**: Information on the points described in the theory section, articles, websites with information, etc.

*Note: If at any point in the exercise description it is required to add mathematical equations, these must be put into the equation format in LaTeX. You can use this [online equation generator](https://www.hostmath.com/) for ease of use. For more information about equations in LaTeX, check their [documentation](https://en.wikibooks.org/wiki/LaTeX/Mathematics). *



## Jekyll Local Installation

### Installing Ruby on Ubuntu

First of all, we need to install all the dependencies typing:

```bash
sudo apt-get install ruby-full build-essential zlib1g-dev
```

After that, we need to set up a gem installation directory for your user account. The following commands will add environment variables to your `~/.bashrc` file to configure the gem installation path. Run them now:

```bash
echo '# Install Ruby Gems to ~/gems' >> ~/.bashrc
echo 'export GEM_HOME="$HOME/gems"' >> ~/.bashrc
echo 'export PATH="$HOME/gems/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

Finally, we install Jekyll:

```bash
gem install jekyll bundler
```

Notice that we don't use the `root` user :-)

### Running Jekyll Serve

By default, the Jekyll server is launched with the following command (which is the one indicated on your website).

```bash
bundle exec jekyll serve
```

If in the process of building the server there is a dependency problem, for example, there is a missing library to install, it is **necessary to delete** the `Gemfile.lock` file so that it is rebuilt with the installed dependency. This list of dependencies is found in the `Gemfile` file (in Python it would be equivalent to the `requirements.txt` file) and the version of each of the installed gems (packages) is specified. Having a list of dependencies is important for future updates as well as knowing the libraries needed to run the server. Once the `Gemfile.lock` file is deleted, the command shown above is launched again and the dependency errors should end.

### Installing Ruby and Jekyll on Mac OS X

Follow the Jekyll [page installation guide](https://jekyllrb.com/docs/installation/macos/).


## FAQ

- Error building Jekyll server: 

```bash
jekyll build --incremental --verbose
```
