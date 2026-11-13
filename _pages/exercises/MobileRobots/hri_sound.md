---
permalink: /exercises/HRI/hri_sound/
title: "HRI Sound"

sidebar:
    nav: "docs"

toc: true
toc_label: "TOC HRI Sound"
toc_icon: "cog"

 youtubeId1: utNTaveaVEs
---

## Goal

The goal of this exercise is to practice **Human-Robot Interaction (HRI) through audio**. The robot (the vacuum cleaner of the `basic_vacuum_cleaner` exercise) can **speak** to the user, converting text into audio that is played in the browser, and can **listen** to the user, converting the audio captured by the browser microphone into text.

{% include gallery caption="Gallery" %}

The students will program the robot so that it interacts with the user by voice while it performs its usual tasks in the simulation. As in the `basic_vacuum_cleaner` exercise, the final goal is to cover the largest possible area with the vacuum robot.

## Frequency API

### Python

- `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.


## Robot API

### Python

#### Vacuum cleaner (HAL)

The robot base of this exercise is the one from the `basic_vacuum_cleaner` exercise.

- `import HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `HAL.getBumperData().state` - to establish if the robot has crashed or not. Returns 1 if the robot collides and 0 if it has not crashed. DISABLED: use laser data in the center (90) to get the same result.
- `HAL.getBumperData().bumper` - if the robot has crashed, it returns 1 when the crash occurs on center of the robot, 0 when it occurs on its right and 2 if the collision is on its left. DISABLED: use laser data in the center (90) to get the same result.
- `HAL.getPose3d().x`, `HAL.getPose3d().y`, `HAL.getPose3d().yaw` - to get the current ground-truth position and orientation of the robot.
- `HAL.setV(velocity)` - to set the linear speed.
- `HAL.setW(velocity)` - to set the angular velocity.
- `HAL.getLaserData()` - It allows to obtain the data of the laser sensor, which consists of 180 pairs of values ​​(0-180º, distance in meters).

#### Audio (WebGUI)

- `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to interact with the user through the browser, including the audio ones.

**Audio out (the robot speaks)**

- `WebGUI.say(text)` - converts `text` into speech and plays it in the browser. It is an alias of `WebGUI.play(text)`.
- `WebGUI.play(text)` - same as `say(text)`. If no text is given it plays a default sentence.
- `WebGUI.play_file(file_path)` - plays an audio file (for example an `.mp3` or `.wav`) instead of a synthesized text.
- `WebGUI.stop()` - stops the current playback.
- `WebGUI.is_playing()` - returns `True` if there is audio being synthesized or waiting to be played.
- `WebGUI.get_status()` - returns the state of the audio as a string: `idle`, `synthesizing`, `reading_file`, `playing`, `done`, `stopped` or `error: <reason>`.

**Audio in (the robot listens)**

- `WebGUI.listen(ms)` - returns the text transcribed from the user's voice during the last `ms` milliseconds, as a single string. If the user has not said anything in that period it returns an empty string.

Example of use, combining sound and movement:

```python
import time
import HAL
import WebGUI

WebGUI.say("Hello, I am your vacuum robot. I am ready.")

while True:
    # Enter iterative code!
    HAL.setV(0.3)

    heard = WebGUI.listen(3000)  # what the user said in the last 3 seconds
    if "stop" in heard.lower():
        HAL.setV(0)
        WebGUI.say("Stopping.")
        time.sleep(3)
```

## Theory

### Human-Robot Interaction

Human-Robot Interaction (HRI) is the field that studies how people and robots communicate and cooperate. Voice is one of the most natural channels: the robot can inform the user with spoken messages and understand spoken commands, without screens or buttons.

### Text-to-Speech (TTS)

Text-to-Speech is the process of turning written text into audio. In this exercise the synthesis is done on the server side with [Piper](https://github.com/rhasspy/piper), and the voice used is the English model `en_GB-alan-medium`. The generated audio is sent to the browser, which plays it.

### Speech-to-Text (STT)

Speech-to-Text is the opposite process: converting the audio of a person speaking into text. The listening part of this exercise combines:

- **Voice Activity Detection (VAD)** with [Silero VAD](https://github.com/snakers4/silero-vad), to detect when the user is speaking and when they stop.
- **Transcription** with [faster-whisper](https://github.com/SYSTRAN/faster-whisper), a fast implementation of the Whisper model.


## Hints

Simple hints provided to help you solve the HRI_sound exercise.

### Speaking

- Each new call to `say()` or `play()` **interrupts** the one that is currently being played. If you want to say several sentences in a row, wait until the previous one has finished with `WebGUI.is_playing()` (or by checking that `WebGUI.get_status()` is `done`).
- Do not call `say()` on every iteration of the loop, because the sentence would restart all the time and never be finished. Say it once, or only when something changes. Control how often you iterate with `time.sleep()` or `Frequency.tick()`.
- Keep the messages short, so the user can follow them while the robot moves.

### Listening

- The speech is split into phrases of **at most 3 seconds**. A very long sentence is cut and transcribed in several parts, so use short commands.
- The transcription is not instantaneous and may contain mistakes. Compare the text with keywords (`"stop" in heard.lower()`) instead of looking for an exact match, and handle the case in which `listen()` returns an empty string.
- `listen(ms)` returns what was heard in the last `ms` milliseconds. If you call it every iteration with a big `ms`, you will receive the same phrase more than once: store what you have already processed or use a small window.
- The robot can hear its own voice through the speakers. If it reacts to itself, avoid listening while it is speaking (`WebGUI.is_playing()`).

### Combining sound and navigation

- The vacuum cleaner keeps moving while it speaks and listens, so the main loop should never be blocked for a long time. Check the bumper and laser data in every iteration, as in the `basic_vacuum_cleaner` exercise.
- A simple approach is a state machine (for example *cleaning*, *stopped* and *talking*) in which the spoken commands change the state.

## Example video

{% include youtubePlayer.html id=page.youtubeId1 %}

## Contributors

- Contributors: [Anass Chikou](https://github.com/AnassC7) and [Álvaro Gómez-Pimpollo](https://github.com/algomezpimpoll)

## References

1. [https://github.com/rhasspy/piper](https://github.com/rhasspy/piper)
2. [https://github.com/snakers4/silero-vad](https://github.com/snakers4/silero-vad)
3. [https://github.com/SYSTRAN/faster-whisper](https://github.com/SYSTRAN/faster-whisper)
