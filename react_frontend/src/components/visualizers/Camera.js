import React, { useEffect, useReducer, useRef, useState } from "react";
import styles from "./../../styles/camera_driver/camera_driver.module.css";
import {
  CameraMonitorIcon,
  CameraNotFoundIcon,
} from "../../styles/camera_driver/CameraDriverIcons";

function decode_utf8(s) {
  return decodeURIComponent(escape(s));
}
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

  latency: 0,
  fps: 0,
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
    case "updateLatency":
      return { ...state, latency: action.payload.latency };
    case "updateFps":
      return { ...state, fps: action.payload.fps };
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
  const streamRef = useRef(null); // use useRef to manage the stream
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

  // Function to capture a frame from the video and convert it to a CV_8UC4 matrix
  const captureFrame = () => {
    const video = videoRef.current;
    const canvas = document.createElement("canvas");
    const ctx = canvas.getContext("2d");

    if (video && canvas && ctx) {
      // Set the canvas size equal to the video size
      canvas.width = 320;
      canvas.height = 240;

      // Draw the video frame on the canvas
      ctx.drawImage(video, 0, 0, canvas.width, canvas.height);

      // Get the image data (RGBA pixel array)
      const imageDataURL = canvas.toDataURL("image/jpeg");

      const performance_t = performance.now();
      const time = performance_t
        .toFixed(5)
        .toString()
        .padStart(timeFrameSize, "0");
      // Encode in base64
      // Send the array via WebSocket
      window.RoboticsExerciseComponents.commsManager.send(
        "gui",
        `pick${imageDataURL}${time}`
      );
    }
  };

  // Get the camera stream
  useEffect(() => {
    if (!isVisualReady) return;

    const startCamera = () => {
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
            
            // Set the stream and assign it to the video
            if (videoRef.current) {
              videoRef.current.srcObject = stream;
              streamRef.current = stream; // save the stream in the reference
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
    // Clear stream when component is unmounted
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
        captureFrame();
        dispatch({
          type: "udpateStartTime",
          payload: { startTime: performance.now() },
        });
      } else if (message.data.state === "paused") {
        dispatch({ type: "cameraPause", payload: true });

        dispatch({ type: "updateFps", payload: { fps: 0 } });

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
      if (message.data.update.image) {
        let image_data = JSON.parse(message.data.update.image);
        let source = decode_utf8(image_data.image);

        if (source.length > 0)
          setImageData(`data:image/jpeg;base64,${source}`);

        // Send the ACK of the msg
        // window.RoboticsExerciseComponents.commsManager.send("gui", "ack");
      }

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
          dispatch({ type: "updateFps", payload: { fps } });

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

        // udpate statics on front end after 500ms
        setTimeout(() => {
          dispatch({
            type: "updateLatency",
            payload: { latency: latency.toFixed(0) },
          });
        }, 500);
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
    fps,
    startTime,
    countFrames,
  ]);

  return (
    <div className={styles.camera_container}>
      {/* Contenedor de la primera imagen (video) */}
      <div className={styles.video_section}>
        {/* When get any error from webRtc Camera */}
        {!isCameraReady && (
          <div className={styles.camera_error}>
            <CameraNotFoundIcon />
            {msg.length > 0 && (
              <h3 className={styles.camera_error_msg}>{msg}</h3>
            )}
          </div>
        )}
        <video ref={videoRef} autoPlay className={styles.camera_video} />
      </div>

      {/* Contenedor flex-container con fondo blanco */}
      <div className={styles.camera_partition}>
        {/* Este es el contenedor intermedio que separa las dos imágenes, ahora con fondo blanco */}
      </div>

      {/* Contenedor de la segunda imagen */}
      <div className={styles.camera_output_section}>
        <div
          className={styles.camera_static_section}
          onClick={() =>
            dispatch({ type: "showCameraStatics", payload: !showCameraStatics })
          }
        >
          {!showCameraStatics ? (
            <div className={styles.camera_static_icon}>
              <CameraMonitorIcon cssClass="icon_color" />
            </div>
          ) : (
            <div className={styles.camera_static_box}>
              <div className={styles.camera_static_fps}>
                <p>{fps < 10 ? (fps === 0 ? `0` : `0${fps}`) : fps}</p>
                <span>FPS</span>
              </div>
              <div className={styles.camera_static_lat}>
                <div>
                  <p>
                    {latency >= 1000 ? (latency / 1000).toFixed(0) : latency}
                  </p>
                  <span> {latency >= 1000 ? ` s` : ` ms`}</span>
                </div>
                <span>LAT.</span>
              </div>
            </div>
          )}
        </div>
        <div>
          {imageData && (
            <img src={imageData} alt="Imagen" className={styles.output_image} />
          )}
        </div>
      </div>
    </div>
  );
};

export default Camera;
