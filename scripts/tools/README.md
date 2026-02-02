# YouTube Frame Extractor
Extract frames from YouTube videos for Computer Vision exercises in RoboticsAcademy.

## Purpose
This tool allows students to:
- Extract frames from YouTube videos containing robotics/computer vision content
- Use real-world video data for testing CV algorithms
- Avoid downloading entire videos (saves disk space)

## Installation
Install required dependencies:
```bash
pip install yt-dlp opencv-python

## Usage
Basic Usage
python youtube_frame_extractor.py "https://www.youtube.com/watch?v=VIDEO_ID"

## Advanced Options
python youtube_frame_extractor.py "https://www.youtube.com/watch?v=VIDEO_ID" \
    --output my_frames \
    --interval 60 \
    --max-frames 50

## Parameters
url (required): YouTube video URL
--output or -o: Output folder name (default: frames)
--interval or -i: Extract every Nth frame (default: 30)
--max-frames or -m: Maximum frames to extract (default: 100)

## Examples
Extract 100 frames (every 30th frame):
bash
python youtube_frame_extractor.py "https://www.youtube.com/watch?v=dQw4w9WgXcQ"
Extract 50 frames (every 60th frame) to custom folder:
bash
python youtube_frame_extractor.py "https://www.youtube.com/watch?v=dQw4w9WgXcQ" -o robot_frames -i 60 -m 50

## Output
Frames are saved as JPEG images:
frame_0000.jpg
frame_0001.jpg
frame_0002.jpg
...

## Use Cases
Testing color detection algorithms
Training machine learning models
Analyzing robot movement patterns
Computer vision exercise datasets

## Related Issue
Addresses GitHub Issue #3058