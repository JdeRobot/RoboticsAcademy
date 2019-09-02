---
permalink: /contribute/
title: "Contribute"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC installation"
toc_icon: "cog"
---

This section describes the steps to include an exercise in the Robotics-Academy documentation.

The objective of having documentation in this format is to simplify the developer having to add information from their software, speeding up the process and showing results that fit the structure of Robotics-Academy.


## Where do I attach the information?

By default, the information added to the documentation is split in two:

- A `.md` file with the description of the exercise, steps for execution, comments, etc.
- A folder with resources. Typically images and videos that are associated to the exercise.

The file with the information is placed in the path: `/_portfolio/` with the name of the exercise and with Markdown extension (`.md`).

The images associated to this file with the information are stored in the path: `/assets/images/exercises/<exercise_name>/`

## Name policy

The name policy is simple:

All file names attached to the documentation will be **lowercase** and spaces will be replaced by **underscore** ("`_`") using ASCII characters.

## Images policy

In order to maintain coherence between all the exercises, it is necessary to distinguish between one image and the rest. This is the image shown in the set of exercises. 

The image name policy is `<exercise_name>_teaser`. Note how it has to end in `_teaser`.

The rest of the images have no name restriction. When in doubt, they are named as the exercise and with a number behind them.



