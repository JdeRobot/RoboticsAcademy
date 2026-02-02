#!/usr/bin/env python3
"""
YouTube Frame Extractor Tool for RoboticsAcademy, Extracts frames from YouTube videos for Computer Vision exercises
"""

import cv2
import argparse
import os
import sys

try:
    import yt_dlp
except ImportError:
    print("ERROR: yt-dlp not installed. Install with: pip install yt-dlp")
    sys.exit(1)


def extract_frames_from_youtube(
    url, output_folder="frames", frame_interval=30, max_frames=100
):
    """
    Extract frames from a YouTube video
    Args:
        url (str): YouTube video URL
        output_folder (str): Folder to save extracted frames
        frame_interval (int): Extract every Nth frame
        max_frames (int): Maximum number of frames to extract
    Returns:
        int: Number of frames extracted
    """
    # Create output folder if it doesn't exist
    os.makedirs(output_folder, exist_ok=True)
    print(f"Fetching video from: {url}")
    # Get video stream URL using yt-dlp
    ydl_opts = {"format": "best[ext=mp4]", "quiet": True}
    try:
        with yt_dlp.YoutubeDL(ydl_opts) as ydl:
            info = ydl.extract_info(url, download=False)
            video_url = info["url"]
            video_title = info.get("title", "video")
        print(f"Video: {video_title}")
        print(f"Extracting frames (every {frame_interval} frames)...")
        # Open video with OpenCV
        cap = cv2.VideoCapture(video_url)
        if not cap.isOpened():
            print("ERROR: Could not open video stream")
            return 0
        frame_count = 0
        saved_count = 0
        while cap.isOpened() and saved_count < max_frames:
            ret, frame = cap.read()
            if not ret:
                break
            # Save every Nth frame
            if frame_count % frame_interval == 0:
                frame_filename = os.path.join(
                    output_folder, f"frame_{saved_count:04d}.jpg"
                )
                cv2.imwrite(frame_filename, frame)
                saved_count += 1
                if saved_count % 10 == 0:
                    print(f"Extracted {saved_count} frames...")
            frame_count += 1
        cap.release()
        print(f"\n✓ Successfully extracted {saved_count} frames to '{output_folder}/'")
        return saved_count
    except Exception as e:
        print(f"ERROR: {e}")
        return 0


def main():
    parser = argparse.ArgumentParser(
        description="Extract frames from YouTube videos for Computer Vision exercises"
    )
    parser.add_argument("url", help="YouTube video URL")
    parser.add_argument(
        "--output",
        "-o",
        default="frames",
        help="Output folder for frames (default: frames)",
    )
    parser.add_argument(
        "--interval",
        "-i",
        type=int,
        default=30,
        help="Extract every Nth frame (default: 30)",
    )
    parser.add_argument(
        "--max-frames",
        "-m",
        type=int,
        default=100,
        help="Maximum frames to extract (default: 100)",
    )
    args = parser.parse_args()
    print("=" * 60)
    print("RoboticsAcademy - YouTube Frame Extractor")
    print("=" * 60)
    frames_extracted = extract_frames_from_youtube(
        args.url, args.output, args.interval, args.max_frames
    )
    if frames_extracted > 0:
        print(f"\nFrames saved to: {os.path.abspath(args.output)}")
    else:
        print("\nFailed to extract frames")
        sys.exit(1)


if __name__ == "__main__":
    main()
