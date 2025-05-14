import React, { useState, useRef, useEffect } from "react";
import { base64Data } from "../visualizers/tmp_base64_data";
// import { Volume2, VolumeX, Maximize, Minimize } from "lucide-react";

const VideoPlayer = ({ videoUrl }) => {
  const videoRef = useRef(null);
  const canvasRef = useRef(null);
  const progressBarRef = useRef(null);
  const [isPlaying, setIsPlaying] = useState(false);
  const [isMuted, setIsMuted] = useState(false);
  const [volume, setVolume] = useState(1);
  const [currentTime, setCurrentTime] = useState(0);
  const [duration, setDuration] = useState(0);
  const [isFullScreen, setIsFullScreen] = useState(false);
  const [showControls, setShowControls] = useState(true);
  const [frames, setFrames] = useState([]);
  const controlsTimeoutRef = useRef(null);
  const containerRef = useRef(null);
  const frameIntervalRef = useRef(null);

  useEffect(() => {
    const videoElement = videoRef.current;
    if (!videoElement) return;

    const handleTimeUpdate = () => {
      setCurrentTime(videoElement.currentTime);
    };

    const handleDurationChange = () => {
      setDuration(videoElement.duration);
    };

    const handleEnded = () => {
      setIsPlaying(false);
    };

    videoElement.addEventListener("timeupdate", handleTimeUpdate);
    videoElement.addEventListener("durationchange", handleDurationChange);
    videoElement.addEventListener("ended", handleEnded);

    return () => {
      videoElement.removeEventListener("timeupdate", handleTimeUpdate);
      videoElement.removeEventListener("durationchange", handleDurationChange);
      videoElement.removeEventListener("ended", handleEnded);
    };
  }, []);

  const captureFrame = () => {
    console.log("frame");

    const video = videoRef.current;
    const canvas = canvasRef.current;
    if (!video || !canvas) return;

    canvas.width = 480; //video.videoWidth;
    canvas.height = 320; //video.videoHeight;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
    const frameUrl = canvas.toDataURL("image/jpeg", 1);

    window.RoboticsExerciseComponents.commsManager.send(
      "gui",
      `pick${frameUrl}xxxxxxxxxxxxxxxxxxxx`
    );
  };

  /* test start */

  useEffect(() => {
    if (!isPlaying) return;

    const callback = (message) => {
      // receive ack from gui.py
      if (message.data.update.ack_img === "ack") {
        // call next frame
        captureFrame();
      }
    };
    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    return () => {
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
        callback
      );
    };
  }, [isPlaying]);

  // recived data
  useEffect(() => {
    const imageId = document.getElementById("gui_canvas");
    console.log("gui_canvas ", imageId);
  }, []);
  /* test end */

  useEffect(() => {
    const hideControls = () => {
      if (isPlaying) {
        setShowControls(false);
      }
    };

    if (controlsTimeoutRef.current) {
      window.clearTimeout(controlsTimeoutRef.current);
    }

    if (showControls && isPlaying) {
      controlsTimeoutRef.current = window.setTimeout(hideControls, 3000);
    }

    return () => {
      if (controlsTimeoutRef.current) {
        window.clearTimeout(controlsTimeoutRef.current);
      }
    };
  }, [showControls, isPlaying]);

  const formatTime = (time) => {
    const minutes = Math.floor(time / 60);
    const seconds = Math.floor(time % 60);
    return `${minutes.toString().padStart(2, "0")}:${seconds
      .toString()
      .padStart(2, "0")}`;
  };

  const togglePlay = () => {
    const video = videoRef.current;
    if (!video) return;

    if (isPlaying) {
      video.pause();
    } else {
      video.play();
    }

    setIsPlaying(!isPlaying);
    setShowControls(true);
  };

  const toggleMute = () => {
    const video = videoRef.current;
    if (!video) return;

    video.muted = !video.muted;
    setIsMuted(!isMuted);
  };

  const handleVolumeChange = (e) => {
    const video = videoRef.current;
    if (!video) return;

    const newVolume = parseFloat(e.target.value);
    video.volume = newVolume;
    setVolume(newVolume);
    setIsMuted(newVolume === 0);
  };

  const handleProgressBarClick = (e) => {
    const video = videoRef.current;
    const progressBar = progressBarRef.current;
    if (!video || !progressBar) return;

    const rect = progressBar.getBoundingClientRect();
    const clickPosition = (e.clientX - rect.left) / rect.width;
    const newTime = clickPosition * video.duration;

    video.currentTime = newTime;
    setCurrentTime(newTime);
  };

  const toggleFullScreen = () => {
    const container = containerRef.current;
    if (!container) return;

    if (!document.fullscreenElement) {
      container.requestFullscreen().catch((err) => {
        console.error(
          `Error attempting to enable full-screen mode: ${err.message}`
        );
      });
      setIsFullScreen(true);
    } else {
      document.exitFullscreen();
      setIsFullScreen(false);
    }
  };

  const handleShowControls = () => {
    setShowControls(true);
  };

  return (
    <div className="flex justify-center items-center gap-4">
      <div
        ref={containerRef}
        className="relative  w-full max-w-2xl bg-black rounded-lg overflow-hidden group"
        onMouseMove={handleShowControls}
        onMouseLeave={() => isPlaying && setShowControls(false)}
      >
        <video
          ref={videoRef}
          src={videoUrl}
          className="w-full xh-auto rounded-lg cursor-pointer"
          onClick={togglePlay}
        />

        {/* Hidden canvas for frame capture */}
        <canvas ref={canvasRef} className="hidden" />

        {/* Video controls */}
        <div
          className={`absolute bottom-0 left-0 right-0 bg-gradient-to-t from-black/70 to-transparent px-4 pt-16 pb-2 transition-opacity duration-300 ${
            showControls ? "opacity-100" : "opacity-0"
          }`}
        >
          {/* Progress bar */}
          <div
            ref={progressBarRef}
            className="w-full h-2 bg-gray-600 rounded-full mb-4 cursor-pointer overflow-hidden"
            onClick={handleProgressBarClick}
          >
            <div
              className="h-full bg-blue-500 rounded-full"
              style={{ width: `${(currentTime / duration) * 100}%` }}
            />
          </div>

          <div className="flex items-center justify-between">
            <div className="flex items-center space-x-4">
              {/* Play/Pause button */}
              <button
                onClick={togglePlay}
                className="text-white hover:text-blue-300 transition-colors"
              >
                {isPlaying ? (
                  <Pause className="w-6 h-6" />
                ) : (
                  <Play className="w-6 h-6" />
                )}
              </button>

              {/* Volume control */}
              <div className="flex items-center space-x-2">
                {/* <button
                  onClick={toggleMute}
                  className="text-white hover:text-blue-300 transition-colors"
                >
                  {isMuted ? (
                    <VolumeX className="w-5 h-5" />
                  ) : (
                    <Volume2 className="w-5 h-5" />
                  )}
                </button> */}
                <input
                  type="range"
                  min="0"
                  max="1"
                  step="0.1"
                  value={isMuted ? 0 : volume}
                  onChange={handleVolumeChange}
                  className="w-16 h-1 bg-gray-600 rounded-full appearance-none cursor-pointer [&::-webkit-slider-thumb]:appearance-none [&::-webkit-slider-thumb]:h-3 [&::-webkit-slider-thumb]:w-3 [&::-webkit-slider-thumb]:rounded-full [&::-webkit-slider-thumb]:bg-white"
                />
              </div>

              {/* Time display */}
              <div className="text-white text-sm">
                {formatTime(currentTime)} / {formatTime(duration)}
              </div>
            </div>

            {/* Fullscreen button */}
            {/* <button
              onClick={toggleFullScreen}
              className="text-white hover:text-blue-300 transition-colors"
            >
              {isFullScreen ? (
                <Minimize className="w-5 h-5" />
              ) : (
                <Maximize className="w-5 h-5" />
              )}
            </button> */}
          </div>
        </div>
      </div>
    </div>
  );
};

export default VideoPlayer;

const Play = ({ cssClass }) => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    width="24"
    height="24"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    stroke-width="2"
    stroke-linecap="round"
    stroke-linejoin="round"
    class="lucide lucide-play-icon lucide-play"
    className={cssClass}
  >
    <polygon points="6 3 20 12 6 21 6 3" />
  </svg>
);
const Pause = ({ cssClass }) => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    width="24"
    height="24"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    stroke-width="2"
    stroke-linecap="round"
    stroke-linejoin="round"
    class="lucide lucide-pause-icon lucide-pause"
    className={cssClass}
  >
    <rect x="14" y="4" width="4" height="16" rx="1" />
    <rect x="6" y="4" width="4" height="16" rx="1" />
  </svg>
);
