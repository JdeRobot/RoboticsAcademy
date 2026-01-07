import React, { useState, useRef, useEffect } from "react";
import {
  Box,
  Paper,
  Typography,
  IconButton,
  Slider,
  Select,
  MenuItem,
  Button,
  Stack,
  SelectChangeEvent,
} from "@mui/material";
import {
  PlayArrow,
  Pause,
  VolumeUp,
  VolumeOff,
  VolumeDown,
  Fullscreen,
  UploadFile,
} from "@mui/icons-material";
import CloudUploadRoundedIcon from "@mui/icons-material/CloudUploadRounded";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIContainer from "Components/exercise/WebGUIContainer";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { contrastSelector } from "jderobot-ide-interface";
import {
  StyledVideo,
  StyledVideoContainer,
  StyledVideoInputContainer,
} from "Styles/visualizers/LocalVideo.styles";

// constants
const CANVAS_WIDTH = 320;
const CANVAS_HEIGHT = 240;
const CAPTURE_FPS = 20; // Capture at 20 frames per second
const CAPTURE_INTERVAL_MS = 1000 / CAPTURE_FPS; // 10ms

const Video = () => {
  const theme = useAcademyTheme();

  const [videoFile, setVideoFile] = useState<string | null>(null);
  const [isPlaying, setIsPlaying] = useState<boolean>(false);
  const [currentTime, setCurrentTime] = useState<number>(0);
  const [duration, setDuration] = useState<number>(0);
  const [volume, setVolume] = useState<number>(1);
  const [isMuted, setIsMuted] = useState<boolean>(true);
  const [playbackSpeed, setPlaybackSpeed] = useState<number>(1);
  const [isDragging, setIsDragging] = useState<boolean>(false);
  const [showControls, setShowControls] = useState<boolean>(false);

  // Refs
  const videoRef = useRef<HTMLVideoElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const fileInputRef = useRef<HTMLInputElement>(null);
  const [capturedFrame, setCapturedFrame] = useState<string | null>(null);
  const captureIntervalRef = useRef<number | null>(null);

  const textColor = contrastSelector(
    theme.palette.text,
    theme.palette.darkText,
    theme.palette.bgLight
  );

  // Handle file upload
  const handleFileChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (file && file.type.startsWith("video/")) {
      const url = URL.createObjectURL(file);
      setVideoFile(url);
      setIsPlaying(false);
      setCurrentTime(0);
    }
  };

  // Handle drag and drop
  const handleDrop = (e: React.DragEvent<HTMLDivElement>) => {
    e.preventDefault();
    e.stopPropagation();

    const file = e.dataTransfer.files[0];
    console.log(file);
    if (file && file.type.startsWith("video/")) {
      const url = URL.createObjectURL(file);
      setVideoFile(url);
      setIsPlaying(false);
      setCurrentTime(0);
    }
  };

  const handleDragOver = (e: React.DragEvent<HTMLDivElement>) => {
    e.preventDefault();
    e.stopPropagation();
  };

  // Play/Pause toggle
  const togglePlay = () => {
    if (videoRef.current) {
      if (isPlaying) {
        videoRef.current.pause();
      } else {
        videoRef.current.play();
      }
      setIsPlaying(!isPlaying);
    }
  };

  // Update time
  const handleTimeUpdate = () => {
    if (videoRef.current && !isDragging) {
      setCurrentTime(videoRef.current.currentTime);
    }
  };

  // Load video metadata
  const handleLoadedMetadata = () => {
    if (videoRef.current) {
      setDuration(videoRef.current.duration);
      videoRef.current.muted = true; // Mute by default
    }
  };

  // Seek video
  const handleSeek = (_: Event, newValue: number | number[]) => {
    const seekTime = newValue as number;
    setCurrentTime(seekTime);
    if (videoRef.current) {
      videoRef.current.currentTime = seekTime;
    }
  };

  // Volume control
  const handleVolumeChange = (_: Event, newValue: number | number[]) => {
    const newVolume = newValue as number;
    setVolume(newVolume);
    if (videoRef.current) {
      videoRef.current.volume = newVolume;
    }
    setIsMuted(newVolume === 0);
  };

  // Mute toggle
  const toggleMute = () => {
    if (videoRef.current) {
      videoRef.current.muted = !isMuted;
      setIsMuted(!isMuted);
    }
  };

  // Playback speed
  const handleSpeedChange = (event: SelectChangeEvent<number>) => {
    const speed = Number(event.target.value);
    setPlaybackSpeed(speed);
    if (videoRef.current) {
      videoRef.current.playbackRate = speed;
    }
  };

  // Fullscreen
  const toggleFullscreen = () => {
    if (videoRef.current) {
      if (document.fullscreenElement) {
        document.exitFullscreen();
      } else {
        videoRef.current.requestFullscreen();
      }
    }
  };

  // Format time
  const formatTime = (time: number): string => {
    if (isNaN(time)) return "0:00";
    const minutes = Math.floor(time / 60);
    const seconds = Math.floor(time % 60);
    return `${minutes}:${seconds.toString().padStart(2, "0")}`;
  };

  // Clean up object URL
  useEffect(() => {
    return () => {
      if (videoFile) {
        URL.revokeObjectURL(videoFile);
      }
    };
  }, [videoFile]);

  // Capture current video frame
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);
  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const captureFrame = () => {
    if (videoRef.current && canvasRef.current) {
      const video = videoRef.current;
      const canvas = canvasRef.current;

      canvas.width = CANVAS_WIDTH;
      canvas.height = CANVAS_HEIGHT;

      const ctx = canvas.getContext("2d");
      if (ctx) {
        ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
        const imageDataUrl = canvas.toDataURL("image/jpeg");
        setCapturedFrame(imageDataUrl);

        // Here you can send the imageDataUrl via WebSocket or any other method
        const performance_t = performance.now();
        const time = performance_t
          .toFixed(5)
          .toString()
          .padStart(CAPTURE_FPS, "0");
        // Codificamos en base64
        // Enviar la matriz por WebSocket
        if (manager !== null) {
          manager.send("gui", `pick${imageDataUrl}${time}`);
        }
      }
    }
  };

  // Frame capture interval - 20 FPS when playing
  useEffect(() => {
    if (isPlaying && videoFile) {
      // Start capturing frames
      captureIntervalRef.current = window.setInterval(() => {
        captureFrame();
      }, CAPTURE_INTERVAL_MS);
    } else {
      // Stop capturing when paused
      if (captureIntervalRef.current) {
        clearInterval(captureIntervalRef.current);
        captureIntervalRef.current = null;
      }
    }

    // Cleanup on unmount
    return () => {
      if (captureIntervalRef.current) {
        clearInterval(captureIntervalRef.current);
      }
    };
  }, [isPlaying, videoFile]);

  return (
    <WebGUIContainer>
      {!videoFile ? (
        <StyledVideoInputContainer
          onDrop={handleDrop}
          onDragOver={handleDragOver}
          onClick={() => fileInputRef.current?.click()}
          color={theme.palette.primary}
        >
          <Box
            className="upload-bg"
            sx={{
              position: "absolute",
              inset: 0,
              opacity: 0,
              transition: "opacity 0.3s ease",
              width: "100%",
              height: "100%",
            }}
          />

          <Box sx={{ position: "relative", zIndex: 1 }}>
            <CloudUploadRoundedIcon
              htmlColor={theme.palette.primary}
              sx={{
                fontSize: 80,
                mb: 3,
                animation: "float 3s ease-in-out infinite",
                "@keyframes float": {
                  "0%, 100%": { transform: "translateY(0)" },
                  "50%": { transform: "translateY(-10px)" },
                },
              }}
            />
            <Typography
              variant="h4"
              component="h3"
              sx={{
                fontWeight: "bold",
                mb: 2,
                background: theme.palette.primary,
                WebkitBackgroundClip: "text",
                WebkitTextFillColor: "transparent",
              }}
            >
              Upload Your Video
            </Typography>
            <Typography
              variant="h6"
              color={textColor}
              sx={{ mb: 1, fontWeight: "normal" }}
            >
              Click to browse a video file
            </Typography>
            <Typography variant="body2" color={textColor} sx={{ opacity: 0.7 }}>
              Supports MP4, MKV
            </Typography>
          </Box>
          <input
            ref={fileInputRef}
            type="file"
            accept=".mp4,.mkv,video/mp4,video/x-matroska"
            onChange={handleFileChange}
            style={{ display: "none" }}
          />
        </StyledVideoInputContainer>
      ) : (
        <StyledVideoContainer
          onMouseEnter={() => setShowControls(true)}
          onMouseLeave={() => setShowControls(false)}
        >
          {/* Hidden canvas for frame capture */}
          <canvas ref={canvasRef} style={{ display: "none" }} />
          {/* Original video */}
          <StyledVideo
            ref={videoRef}
            src={videoFile}
            onTimeUpdate={handleTimeUpdate}
            onLoadedMetadata={handleLoadedMetadata}
            onEnded={() => setIsPlaying(false)}
          />

          <Box
            className="controls"
            sx={{
              position: "absolute",
              bottom: 0,
              left: 0,
              right: 0,
              background:
                "linear-gradient(to top, rgba(0, 0, 0, 0.9) 0%, rgba(0, 0, 0, 0.7) 50%, transparent 100%)",
              p: 3,
              opacity: showControls || !isPlaying ? 1 : 0,
              transition: "opacity 0.3s ease",
            }}
          >
            {/* Progress Bar */}
            <Slider
              value={currentTime}
              min={0}
              max={duration || 0}
              onChange={handleSeek}
              onMouseDown={() => setIsDragging(true)}
              onChangeCommitted={() => setIsDragging(false)}
              sx={{
                color: theme.palette.primary,
                height: 6,
                mb: 1,
                "& .MuiSlider-thumb": {
                  width: 0,
                  height: 0,
                  transition: "all 0.2s",
                  "&:hover, &.Mui-focusVisible, &.Mui-active": {
                    width: 16,
                    height: 16,
                    boxShadow: "0 0 0 8px rgba(102, 126, 234, 0.16)",
                  },
                },
                "&:hover .MuiSlider-thumb": {
                  width: 16,
                  height: 16,
                },
                "& .MuiSlider-rail": {
                  opacity: 0.3,
                  backgroundColor: "#fff",
                },
              }}
            />

            {/* Control Buttons */}
            <Stack
              direction="row"
              alignItems="center"
              justifyContent="space-between"
              spacing={2}
            >
              <Stack direction="row" alignItems="center" spacing={2}>
                {/* Play/Pause */}
                <IconButton
                  onClick={togglePlay}
                  sx={{
                    color: "#fff",
                    "&:hover": {
                      color: theme.palette.primary,
                      bgcolor: "rgba(255,255,255,0.1)",
                    },
                  }}
                >
                  {isPlaying ? <Pause /> : <PlayArrow />}
                </IconButton>

                {/* Volume */}
                <Stack
                  direction="row"
                  alignItems="center"
                  spacing={1}
                  sx={{ width: 150 }}
                >
                  <IconButton
                    onClick={toggleMute}
                    size="small"
                    sx={{
                      color: "#fff",
                      "&:hover": { color: theme.palette.primary },
                    }}
                  >
                    {isMuted || volume === 0 ? (
                      <VolumeOff />
                    ) : volume < 0.5 ? (
                      <VolumeDown />
                    ) : (
                      <VolumeUp />
                    )}
                  </IconButton>
                  <Slider
                    value={isMuted ? 0 : volume}
                    min={0}
                    max={1}
                    step={0.01}
                    onChange={handleVolumeChange}
                    sx={{
                      color: theme.palette.primary,
                      height: 4,
                      "& .MuiSlider-thumb": {
                        width: 12,
                        height: 12,
                      },
                    }}
                  />
                </Stack>

                {/* Time */}
                <Typography
                  variant="body2"
                  sx={{
                    color: "#fff",
                    minWidth: 100,
                    fontVariantNumeric: "tabular-nums",
                  }}
                >
                  {formatTime(currentTime)} / {formatTime(duration)}
                </Typography>
              </Stack>

              <Stack direction="row" alignItems="center" spacing={2}>
                {/* Speed Control */}
                <Select
                  value={playbackSpeed}
                  onChange={handleSpeedChange}
                  variant="standard"
                  disableUnderline
                  sx={{
                    color: "#fff",
                    fontSize: "0.875rem",
                    "& .MuiSelect-icon": { color: "#fff" },
                    "&:hover": { color: theme.palette.primary },
                  }}
                >
                  <MenuItem value={0.5}>0.5x</MenuItem>
                  <MenuItem value={0.75}>0.75x</MenuItem>
                  <MenuItem value={1}>1x</MenuItem>
                  <MenuItem value={1.25}>1.25x</MenuItem>
                  <MenuItem value={1.5}>1.5x</MenuItem>
                  <MenuItem value={2}>2x</MenuItem>
                </Select>

                {/* Fullscreen */}
                <IconButton
                  onClick={toggleFullscreen}
                  sx={{
                    color: "#fff",
                    "&:hover": {
                      color: theme.palette.primary,
                      bgcolor: "rgba(255,255,255,0.1)",
                    },
                  }}
                >
                  <Fullscreen />
                </IconButton>
              </Stack>
            </Stack>
          </Box>

          {/* Change Video Button */}
          <Button
            variant="contained"
            startIcon={<UploadFile />}
            onClick={() => {
              setVideoFile(null);
              setIsPlaying(false);
              setCurrentTime(0);
            }}
            sx={{
              position: "absolute",
              top: 24,
              right: 24,
              bgcolor: "rgba(255,255,255,0.1)",
              backdropFilter: "blur(10px)",
              opacity: showControls || !isPlaying ? 1 : 0,
              transition: "all 0.3s ease",
              "&:hover": {
                bgcolor: theme.palette.primary,
                transform: "translateY(-2px)",
                boxShadow: "0 0 20px rgba(102, 126, 234, 0.3)",
              },
            }}
          >
            Change Video
          </Button>
        </StyledVideoContainer>
      )}
    </WebGUIContainer>
  );
};

export default Video;
