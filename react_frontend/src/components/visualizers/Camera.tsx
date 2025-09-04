import React, { useEffect, useReducer, useRef, useState } from "react";
import { Box } from "@mui/system";

type CameraState = {
  isCameraReady: boolean;
  isCameraPause: boolean;
  isVisualReady: boolean;
  countFrames: number;
  startTime: number;
};

import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import { StyledCameraError, StyledWebCamVideo } from "Styles/camera_driver/Camera.styles";
import { useTheme } from "jderobot-ide-interface";
import VideocamOffOutlinedIcon from '@mui/icons-material/VideocamOffOutlined';

type CameraAction =
  | { type: "cameraReady"; payload: boolean }
  | { type: "cameraPause"; payload: boolean }
  | { type: "visiualReady"; payload: boolean }
  | { type: "updateCountFrames"; payload: { countFrames: number } }
  | { type: "udpateStartTime"; payload: { startTime: number } };

// webRTC error message
const cameraErrorMessages: Record<string, string> = {
  NotAllowedError: "Camera access denied by the user.",
  OverconstrainedError:
    "The camera is already being used by another application or tab",
  NotFoundError: "No media devices found.",
  DevicesNotFoundError: "No media devices found.",
};

// reducer initial state
const initialState: CameraState = {
  isCameraReady: false,
  isCameraPause: false,
  isVisualReady: false,
  countFrames: 0,
  startTime: 0,
};
// reducer func
const reducer = (state: CameraState, action: CameraAction): CameraState => {
  switch (action.type) {
    case "cameraReady":
      return { ...state, isCameraReady: action.payload };
    case "cameraPause":
      return { ...state, isCameraPause: action.payload };
    case "visiualReady":
      return { ...state, isVisualReady: action.payload };
    case "updateCountFrames":
      return { ...state, countFrames: action.payload.countFrames };
    case "udpateStartTime":
      return { ...state, startTime: action.payload.startTime };

    default:
      return state;
  }
};

// time frame size
const timeFrameSize = 20;

// camera
const Camera = () => {
  const exerciseContext = useExercise();  
  const theme = useTheme();
  const [manager, setManager] = useState(exerciseContext.manager);
  const [state, setState] = useState<string>("Connecting to media device.");
  const [mediaStream, setMediaStream] = useState<MediaStream | null>(null);
  const videoRef = useRef<HTMLVideoElement>(null);
  const [
    { isCameraReady, isCameraPause, isVisualReady, countFrames, startTime },
    dispatch,
  ] = useReducer(reducer, initialState);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  useEffect(() => {
    if (isVisualReady) {
      startWebcam();
    }
    return () => {
      if (isVisualReady) {
        stopWebcam();
      }
    };
  }, [isVisualReady]);

  const startWebcam = async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({
        video: {
          facingMode: "user", // Request the front camera (selfie camera)
        },
      });
      if (videoRef.current) {
        videoRef.current.srcObject = stream;
      }
      setMediaStream(stream);
    } catch (error: any) {
      const errorMessage = cameraErrorMessages[error.name];
      setState(errorMessage ? errorMessage : `Something went wrong!`);
      console.error("Error accessing webcam", error);
    }
  };

  const stopWebcam = () => {
    if (mediaStream) {
      mediaStream.getTracks().forEach((track) => {
        track.stop();
      });
      setMediaStream(null);
    }
  };

  // Función para capturar un fotograma del video y convertirlo en una matriz CV_8UC4
  const captureFrame = () => {
    const video = videoRef.current;
    const canvas = document.createElement("canvas");
    const ctx = canvas.getContext("2d");

    if (video && canvas && ctx) {
      // Establecer el tamaño del canvas igual al tamaño del video
      canvas.width = 320;
      canvas.height = 240;

      // Dibujar el frame del video en el canvas
      ctx.drawImage(video, 0, 0, canvas.width, canvas.height);

      // Obtener los datos de la imagen (array de píxeles RGBA)
      const imageDataURL = canvas.toDataURL("image/jpeg");

      const performance_t = performance.now();
      const time = performance_t
        .toFixed(5)
        .toString()
        .padStart(timeFrameSize, "0");
      // Codificamos en base64
      // Enviar la matriz por WebSocket
      if (manager !== null) {
        manager.send("gui", `pick${imageDataURL}${time}`);
      }
    }
  };

  // handle and udpate camera state, depending on RAM state
  useEffect(() => {
    if (manager === null) {
      return;
    }

    const stateCallback = (message: MessageEvent<any>) => {
      console.log(message);
      if (message.data.state === "tools_ready") {
        dispatch({ type: "visiualReady", payload: true });
      }
      if (message.data.state === "application_running") {
        dispatch({ type: "cameraPause", payload: false });
        dispatch({
          type: "udpateStartTime",
          payload: { startTime: performance.now() },
        });
      } else if (message.data.state === "paused") {
        dispatch({ type: "cameraPause", payload: true });

        manager.send("gui", `introspection:${0}/${0}`);

        dispatch({
          type: "updateCountFrames",
          payload: { countFrames: 0 },
        });
      }
    };

    const updateCallback = (message: MessageEvent<any>) => {
      if (message.data.update.ack_img === "ack" && !isCameraPause) {
        captureFrame(); // call next frame

        const prevTime = Number(message.data.update.time);
        const currTime = performance.now();
        const latency = currTime - prevTime;

        //count frames
        dispatch({
          type: "updateCountFrames",
          payload: { countFrames: countFrames + 1 },
        });

        const elapsedTime = currTime - startTime;
        // udpate after 1s
        if (elapsedTime >= 1000) {
          const fps = Math.ceil(countFrames / (elapsedTime / 1000)).toFixed(0);

          // udpate fps
          manager.send("gui", `introspection:${fps}/${latency.toFixed(0)}`);

          // reset count frames
          dispatch({
            type: "updateCountFrames",
            payload: { countFrames: 0 },
          });

          // reset start time
          dispatch({
            type: "udpateStartTime",
            payload: { startTime: currTime },
          });
        }
      }
    };

    manager.subscribe(events.STATE_CHANGED, stateCallback);

    if (isVisualReady && isCameraReady) {
      manager.subscribe(events.UPDATE, updateCallback);
    }

    return () => {
      manager.unsubscribe(events.STATE_CHANGED, stateCallback);

      if (isVisualReady && isCameraReady) {
        manager.unsubscribe(events.UPDATE, updateCallback);
      }
    };
  }, [
    manager,
    isCameraPause,
    isVisualReady,
    isCameraReady,
    startTime,
    countFrames,
  ]);

  return (
    <Box
      sx={{
        display: "flex",
        justifyContent: "center",
        alignItems: "center",
        maxHeight: "100%",
        width: "100%",
        height: "100%",
        textAlign: "center",
      }}
    >
      {mediaStream === null && (
        <StyledCameraError color={theme.palette.error}>
          <VideocamOffOutlinedIcon htmlColor={theme.palette.darkText} />
          {state.length > 0 && <h3>{state}</h3>}
        </StyledCameraError>
      )}
      <StyledWebCamVideo
        ref={videoRef}
        autoPlay
        visible={mediaStream !== null}
      />
    </Box>
  );
};

export default Camera;
