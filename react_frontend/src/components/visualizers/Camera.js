import React, { useEffect, useReducer, useRef } from "react";
import styles from "./../../styles/camera_driver/camera_driver.module.css";
import {
  CameraNotFoundIcon,
} from "../../styles/camera_driver/CameraDriverIcons";
import { Box } from "@mui/system";

// webRTC error message
const cameraErrorMessages = {
  NotAllowedError: "Camera access denied by the user.",
  OverconstrainedError:
    "The camera is already being used by another application or tab",
  NotFoundError: "No media devices found.",
  DevicesNotFoundError: "No media devices found.",
};

// reducer initial state
const initialState = {
  isCameraReady: false,
  isCameraPause: false,
  isVisualReady: false,
  showCameraStatics: false,

  countFrames: 0,
  startTime: 0,

  msg: "Connecting to media device.",
};
// reducer func
const reducer = (state, action) => {
  switch (action.type) {
    case "cameraReady":
      return { ...state, isCameraReady: action.payload };
    case "cameraPause":
      return { ...state, isCameraPause: action.payload };
    case "visiualReady":
      return { ...state, isVisualReady: action.payload };
    case "showCameraStatics":
      return { ...state, showCameraStatics: action.payload };
    case "updateCountFrames":
      return { ...state, countFrames: action.payload.countFrames };
    case "udpateStartTime":
      return { ...state, startTime: action.payload.startTime };
    case "udpateMsg":
      return { ...state, msg: action.payload.msg };

    default:
      return state;
  }
};

// time frame size
const timeFrameSize = 20;

// camera
const Camera = () => {
  const commsManager = window.RoboticsExerciseComponents.commsManager;
  const videoRef = useRef(null);
  const streamRef = useRef(null); // Usamos useRef para manejar el stream
  const [imageData, setImageData] = React.useState("");

  // reducer
  const [
    {
      isCameraReady,
      isCameraPause,
      isVisualReady,
      showCameraStatics,
      latency,
      fps,
      countFrames,
      startTime,
      msg,
    },
    dispatch,
  ] = useReducer(reducer, initialState);

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
      window.RoboticsExerciseComponents.commsManager.send(
        "gui",
        `pick${imageDataURL}${time}`
      );
    }
  };

  // Obtener el stream de la cámara
  useEffect(() => {
    if (!isVisualReady) return;

    const startCamera = () => {
      console.log("Start camera");
      // configure media parameters
      const constraints = {
        video: true,
        audio: false,
      };
      if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
        navigator.mediaDevices
          .getUserMedia(constraints)
          .then((stream) => {
            dispatch({ type: "cameraReady", payload: true });
            dispatch({ type: "udpateMsg", payload: { msg: "" } });
            // Establecer el stream y asignarlo al video

            if (videoRef.current) {
              videoRef.current.srcObject = stream;
              streamRef.current = stream; // Guardamos el stream en la referencia
            }
          })
          .catch((err) => {
            dispatch({ type: "cameraReady", payload: false });

            const errorMessage = cameraErrorMessages[err.name];
            dispatch({
              type: "udpateMsg",
              payload: {
                msg: errorMessage ? errorMessage : `Something went wrong!`,
              },
            });

            console.log(err);
          });
      }
    };

    startCamera();
    // Limpiar el stream cuando el componente se desmonte
    return () => {
      if ((streamRef, current)) {
        streamRef.getTracks().forEach((track) => track.stop());
      }
    };
  }, [isVisualReady]);

  // handle and udpate camera state, depending on RAM state
  useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "visualization_ready") {
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

        window.RoboticsExerciseComponents.commsManager.send(
          "gui",
          `introspection:${0}/${0}`
        );

        dispatch({
          type: "updateCountFrames",
          payload: { countFrames: 0 },
        });
      }
    };
    commsManager.subscribe([commsManager.events.STATE_CHANGED], callback);

    return () => {
      commsManager.unsubscribe([commsManager.events.STATE_CHANGED], callback);
    };
  }, []);

  // ack (you can get response from update_gui() in GUI.py)
  useEffect(() => {
    if (!isVisualReady || !isCameraReady) return;

    const callback = (message) => {
      // receive ack from gui.py
      if (message.data.update.ack_img === "ack" && !isCameraPause) {
        // call next frame
        captureFrame();

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
          window.RoboticsExerciseComponents.commsManager.send(
            "gui",
            `introspection:${fps}/${latency.toFixed(0)}`
          );

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
    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    return () => {
      // console.log("TestShowScreen unsubscribing from ['state-changed'] events");
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
        callback
      );
    };
  }, [
    isCameraPause,
    isVisualReady,
    isCameraReady,
    //
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
      {!isCameraReady && (
        <div className={styles.camera_error}>
          <CameraNotFoundIcon />
          {msg.length > 0 && <h3 className={styles.camera_error_msg}>{msg}</h3>}
        </div>
      )}
      <video ref={videoRef} autoPlay className={styles.camera_video} />
    </Box>
  );
};

export default Camera;
