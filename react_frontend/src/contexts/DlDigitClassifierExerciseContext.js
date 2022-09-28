/* eslint-disable no-unused-vars */
import * as React from "react";
import { createContext, useState, useRef } from "react";
import PropTypes from "prop-types";
import { Typography } from "@mui/material";
import { decode_utf8 } from "../helpers/utils";
const websocket_address = "127.0.0.1";
const address_code = "ws://" + websocket_address + ":1905";
const address_gui = "ws://" + websocket_address + ":2303";
let ws_manager, websocket_code, websocket_gui;
let brainFreqAck = 12,
  guiFreqAck = 12;
let running = true,
  resetRequested = false,
  firstCodeSent = false;
const FileLabel = "Choose a Deep learning model(Onnx Format):";
const acceptExtensions = ".onnx";
let image = new Image();
let operation, shape;
let openConsoleSnackbar = true;
function createData(key, value) {
  return { key, value };
}

const DlDigitClassifierExerciseContext = createContext();

export function ExerciseProvider({ children }) {
  const exercise = "dl_digit_classifier";
  const editorRef = useRef();
  const fileRef = useRef();
  const guiCanvasRef = useRef();
  const exerciseSpecificCSS = "digit_canvas";
  // connectionState - Connect, Connecting, Connected
  const [connectionState, setConnectionState] = useState("Connect");
  // launchState - Launch, Launching, Ready
  const [launchState, setLaunchState] = useState("Launch");
  const [launchLevel, setLaunchLevel] = useState(0);
  const [openInfoModal, setOpenInfoModal] = useState(false);
  const [openErrorModal, setOpenErrorModal] = useState(false);
  const [openLoadModal, setOpenLoadModal] = useState(false);
  const [alertState, setAlertState] = useState({
    errorAlert: false,
    successAlert: false,
    infoAlert: false,
    warningAlert: false,
  });
  const [canvasHeading, setCanvasHeading] = useState("Digit Classifier");
  const [alertContent, setAlertContent] = useState("");
  const [openConsole, setOpenConsole] = useState(false);
  const [playState, setPlayState] = useState(true);
  const [brainFreq, setBrainFreq] = useState(brainFreqAck);
  const [frequencyRows, setFrequencyRows] = useState([
    createData("Brain Frequency (Hz)", 0),
    createData("GUI Frequency (Hz)", 0),
  ]);
  let [errorContentHeading, setErrorContentHeading] =
    useState("Errors detected !");
  let [errorContent, setErrorContent] = useState(
    <Typography> No error detected </Typography>
  );
  const [guiFreq, setGuiFreq] = useState(guiFreqAck);

  const onPageLoad = () => {
    startSim(0);
  };

  const onUnload = () => {
    startSim(2);
    return true;
  };

  const startSim = (step) => {
    var level = 0;
    let websockets_connected = false;
    if (step === 0) {
      setConnectionState("Connecting");
      ws_manager = new WebSocket("ws://" + websocket_address + ":8765/");
    } else if (step === 1) {
      connectionUpdate(
        { connection: "exercise", command: "launch_level", level: `${level}` },
        "*"
      );
      ws_manager.send(
        JSON.stringify({
          command: "open",
          exercise: exercise,
        })
      );
      level++;
      connectionUpdate(
        { connection: "exercise", command: "launch_level", level: `${level}` },
        "*"
      );
      ws_manager.send(JSON.stringify({ command: "Pong" }));
    } else if (step === 2) {
      ws_manager.send(JSON.stringify({ command: "exit", exercise: "" }));
    }

    ws_manager.onopen = function (event) {
      level++;
      connectionUpdate({ connection: "manager", command: "up" }, "*");
      connectionUpdate({ connection: "exercise", command: "available" }, "*");
    };

    ws_manager.onmessage = function (event) {
      //console.log(event.data);
      if (event.data.level > level) {
        level = event.data.level;
        connectionUpdate(
          {
            connection: "exercise",
            command: "launch_level",
            level: `${level}`,
          },
          "*"
        );
      }
      if (event.data.includes("Ping")) {
        if (!websockets_connected && event.data === "Ping3") {
          level = 4;
          connectionUpdate(
            {
              connection: "exercise",
              command: "launch_level",
              level: `${level}`,
            },
            "*"
          );
          websockets_connected = true;
          declare_code(websocket_address);
          declare_gui(websocket_address);
        }
        setTimeout(function () {
          ws_manager.send(JSON.stringify({ command: "Pong" }));
        }, 1000);
      }
    };
  };

  function connectionUpdate(data) {
    if (data.connection === "manager") {
      if (data.command === "up") {
        setConnectionState("Connected");
      } else if (data.command === "down") {
        setConnectionState("Connect");
        if (websocket_code != null) websocket_code.close();
        if (websocket_gui != null) websocket_gui.close();
        setLaunchState("Launch");
      }
    } else if (data.connection === "exercise") {
      if (data.command === "available") {
        // Do nothing
      } else if (data.command === "up") {
        let submit_button = document.getElementById("submit");
        submit_button.disabled = false;
        submit_button.style.opacity = "1.0";
        submit_button.style.cursor = "default";
        setLaunchState("Ready");
        let reset_button = document.getElementById("reset");
        reset_button.disabled = false;
        reset_button.style.opacity = "1.0";
        reset_button.style.cursor = "default";
      } else if (data.command === "down") {
        setLaunchState("Launch");
      } else if (data.command === "launch_level") {
        let level = data.level;
        setLaunchLevel(level);
        setLaunchState("Launching");
      } else if (data.command === "error") {
        setErrorContent(<Typography>{data.text}</Typography>);
        handleErrorModalOpen();
      } else if (data.command === "style") {
        setErrorContentHeading("Style Evaluation ");
        handleErrorModalOpen();

        if (data.text.replace(/\s/g, "").length)
          setErrorContent(<Typography>{data.text}</Typography>);
        else
          setErrorContent(
            <Typography color={"success"}>Everything is correct !</Typography>
          );
      }
    }
  }

  function enablePlayPause(enable) {
    let playPause_button = document.getElementById("submit");
    if (enable === false) {
      playPause_button.disabled = true;
      playPause_button.style.opacity = "0.4";
      playPause_button.style.cursor = "not-allowed";
    } else if (firstCodeSent === true) {
      playPause_button.disabled = false;
      playPause_button.style.opacity = "1.0";
      playPause_button.style.cursor = "default";
    }
  }

  const changeConsole = () => {
    if (openConsoleSnackbar) {
      openConsoleSnackbar = false;
      setAlertState({
        ...alertState,
        errorAlert: false,
        successAlert: false,
        warningAlert: false,
        infoAlert: true,
      });
      setAlertContent(`Console Opened !!`);
    } else {
      openConsoleSnackbar = true;
      setAlertState({
        ...alertState,
        errorAlert: false,
        successAlert: false,
        warningAlert: true,
        infoAlert: false,
      });
      setAlertContent(`Console Closed !!`);
    }
    setOpenConsole(!openConsole);
  };

  // WS_CODE_BEGINS
  function declare_code() {
    websocket_code = new WebSocket("ws://" + websocket_address + ":1905/");

    websocket_code.onopen = function (event) {
      if (websocket_gui.readyState === 1) {
        connectionUpdate({ connection: "exercise", command: "up" }, "*");
        setAlertState({
          ...alertState,
          errorAlert: false,
          successAlert: true,
          warningAlert: false,
          infoAlert: false,
        });
        setAlertContent(" Connection established! ");
      }
      websocket_code.send("#ping");
    };
    websocket_code.onclose = function (event) {
      if (event.wasClean) {
        setAlertState({
          ...alertState,
          errorAlert: false,
          successAlert: false,
          warningAlert: true,
          infoAlert: false,
        });
        setAlertContent(
          `Connection closed cleanly, code=${event.code} reason=${event.reason}`
        );
      } else {
        setAlertState({
          ...alertState,
          errorAlert: false,
          successAlert: false,
          warningAlert: true,
          infoAlert: false,
        });
        setAlertContent(" Connection closed! ");
      }
    };

    websocket_code.onmessage = function (event) {
      var source_code = event.data;
      operation = source_code.substring(0, 5);

      if (operation === "#load") {
        // Nothing
      } else if (operation === "#freq") {
        try {
          let frequency_message = JSON.parse(source_code.substring(5));
          // Parse GUI and Brain frequencies
          const idealGuiFreqValue = frequency_message.gui;
          const idealBrainFreqValue = frequency_message.brain;
          setFrequencyRows([
            createData("Brain Frequency (Hz)", idealBrainFreqValue),
            createData("GUI Frequency (Hz)", idealGuiFreqValue),
          ]);

          frequency_message = {
            brain: brainFreqAck,
            gui: guiFreqAck,
          };
          websocket_code.send("#freq" + JSON.stringify(frequency_message));
        } catch (e) {
          console.error(e);
        }
      } else if (operation === "#ping") {
        websocket_code.send("#ping");
      }
    };
  }
  const submitCode = () => {
    setCanvasHeading("Uploading model...");
    var fr = new FileReader();
    fr.readAsDataURL(fileRef.current.files[0]);
    fr.onload = (event) => {
      websocket_code.send(event.target.result);
    };
    running = true;
  };

  // Function that send/submits an empty string
  const stopCode = () => {
    var stop_inference = "#code\n";
    console.log("Message sent!");
    websocket_code.send(stop_inference);

    running = false;
  };
  // WS_CODE_ENDS

  // WS_GUI_BEGINS
  function declare_gui() {
    websocket_gui = new WebSocket("ws://" + websocket_address + ":2303/");

    websocket_gui.onopen = function (event) {
      if (websocket_code.readyState === 1) {
        setAlertState({
          ...alertState,
          errorAlert: false,
          successAlert: true,
          warningAlert: false,
          infoAlert: false,
        });
        setAlertContent(" Connection established! ");
        connectionUpdate({ connection: "exercise", command: "up" }, "*");
      }
    };

    websocket_gui.onclose = function (event) {
      connectionUpdate({ connection: "exercise", command: "down" }, "*");
      if (event.wasClean) {
        setAlertState({
          ...alertState,
          errorAlert: false,
          successAlert: false,
          warningAlert: true,
          infoAlert: false,
        });
        setAlertContent(
          `Connection closed cleanly, code=${event.code} reason=${event.reason}`
        );
      } else {
        setAlertState({
          ...alertState,
          errorAlert: false,
          successAlert: false,
          warningAlert: true,
          infoAlert: false,
        });
        setAlertContent(`Connection closed!`);
      }
    };

    // What to do when a message from server is received
    websocket_gui.onmessage = function (event) {
      operation = event.data.substring(0, 4);
      if (operation === "#gui") {
        // Parse the entire Object
        let data = JSON.parse(event.data.substring(4));

        // Parse the Image Data
        let image_data = JSON.parse(data.image),
          source = decode_utf8(image_data.image),
          digit = image_data.digit;
        shape = image_data.shape;

        if (source !== "") {
          image.src = "data:image/jpeg;base64," + source;
          guiCanvasRef.current.width = shape[1];
          guiCanvasRef.current.height = shape[0];
        }

        if (digit !== "") {
          setCanvasHeading("Digit found: " + digit);
        }

        // Send the Acknowledgment Message
        websocket_gui.send("#ack");
      }
    };
  }

  // For image object
  image.onload = function () {
    update_image();
  };

  // Request Animation Frame to remove the flickers
  function update_image() {
    window.requestAnimationFrame(update_image);
    const context = guiCanvasRef.current.getContext("2d");
    context.drawImage(image, 0, 0);
  }
  // WS_GUI_ENDS

  const keyHandleFrequency = (e) => {
    if (e.key === "ArrowUp") {
      if (e.target.id === "gui_freq") {
        if (guiFreqAck < 30) {
          guiFreqAck = guiFreqAck + 1;
          setGuiFreq(guiFreqAck);
        }
      }
      if (e.target.id === "code_freq") {
        if (brainFreqAck < 30) {
          brainFreqAck = brainFreqAck + 1;
          setBrainFreq(brainFreqAck);
        }
      }
    }
    if (e.key === "ArrowDown") {
      if (e.target.id === "gui_freq") {
        if (guiFreqAck > 1) {
          guiFreqAck = guiFreqAck - 1;
          setGuiFreq(guiFreqAck);
        }
      }
      if (e.target.id === "code_freq") {
        if (brainFreqAck > 1) {
          brainFreqAck = brainFreqAck - 1;
          setBrainFreq(brainFreqAck);
        }
      }
    }
  };

  const handleInfoModalOpen = () => setOpenInfoModal(true);
  const handleInfoModalClose = () => setOpenInfoModal(false);
  const handleErrorModalOpen = () => setOpenErrorModal(true);
  const handleErrorModalClose = () => setOpenErrorModal(false);
  const handleLoadModalOpen = () => setOpenLoadModal(true);
  const handleLoadModalClose = () => setOpenLoadModal(false);
  const connectionButtonClick = () => {
    if (connectionState === "Connect") {
      setConnectionState("Connecting");
      startSim(0);
    }
  };

  const launchButtonClick = () => {
    if (connectionState === "Connected" && launchState === "Launch") {
      setLaunchState("Launching");
      startSim(1);
    } else if (connectionState === "Connect") {
      setAlertState({
        ...alertState,
        errorAlert: false,
        successAlert: false,
        warningAlert: true,
        infoAlert: false,
      });
      setAlertContent(
        `A connection with the manager must be established before launching an exercise`
      );
    }
  };

  const loadModelButton = (event) => {
    event.preventDefault();
    var fr = new FileReader();
    fr.readAsDataURL(event.target.files[0]);
    setAlertState({
      ...alertState,
      errorAlert: false,
      successAlert: true,
      warningAlert: false,
      infoAlert: false,
    });
    setAlertContent("File Uploaded Successfully");
  };

  const start = () => {
    submitCode();
  };
  return (
    <DlDigitClassifierExerciseContext.Provider
      value={{
        acceptExtensions,
        FileLabel,
        openLoadModal,
        connectionState,
        launchState,
        launchLevel,
        playState,
        openConsole,
        alertState,
        alertContent,
        brainFreq,
        guiFreq,
        frequencyRows,
        openInfoModal,
        openErrorModal,
        errorContent,
        errorContentHeading,
        exerciseSpecificCSS,
        editorRef,
        guiCanvasRef,
        fileRef,
        canvasHeading,
        loadModelButton,
        handleLoadModalOpen,
        handleLoadModalClose,
        connectionButtonClick,
        launchButtonClick,
        stop,
        start,
        changeConsole,
        handleInfoModalOpen,
        handleInfoModalClose,
        handleErrorModalOpen,
        handleErrorModalClose,
        onPageLoad,
        onUnload,
        keyHandleFrequency,
      }}
    >
      {children}
    </DlDigitClassifierExerciseContext.Provider>
  );
}

ExerciseProvider.propTypes = {
  children: PropTypes.node,
};

export default DlDigitClassifierExerciseContext;
