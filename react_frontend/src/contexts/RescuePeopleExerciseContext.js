/* eslint-disable no-unused-vars */
import * as React from "react";
import { createContext, useState, useRef } from "react";
import PropTypes from "prop-types";
import { Typography } from "@mui/material";
import { get_novnc_size_react, saveCode, decode_utf8 } from "../helpers/utils";
const websocket_address = "127.0.0.1";
const address_code = "ws://" + websocket_address + ":1905";
const address_gui = "ws://" + websocket_address + ":2303";
let ws_manager, websocket_code, websocket_gui;
let brainFreqAck = 12,
  guiFreqAck = 12;
let simStop = false,
  sendCode = false,
  running = true,
  firstAttempt = true,
  simReset = false,
  simResume = false,
  resetRequested = false,
  firstCodeSent = false,
  gazeboOn = false,
  gazeboToggle = false;
let code = `from GUI import GUI
from HAL import HAL
# Enter sequential code!

while True:
    # Enter iterative code!`;

function createData(key, value) {
  return { key, value };
}
let image = new Image(),
  image_left = new Image();

const RescuePeopleExerciseContext = createContext();

export function ExerciseProvider({ children }) {
  const exercise = "rescue_people";
  const editorRef = useRef();
  const imageRef = useRef();
  const guiCanvasRef = useRef();
  const guiCanvasRefDrone = useRef();
  // connectionState - Connect, Connecting, Connected
  const [connectionState, setConnectionState] = useState("Connect");
  // launchState - Launch, Launching, Ready
  const [launchState, setLaunchState] = useState("Launch");
  const [launchLevel, setLaunchLevel] = useState(0);
  const [openInfoModal, setOpenInfoModal] = useState(false);
  const [openErrorModal, setOpenErrorModal] = useState(false);
  const [openLoadModal, setOpenLoadModal] = useState(false);
  const [filename, setFilename] = useState("rescue-people");
  const [alertState, setAlertState] = useState({
    errorAlert: false,
    successAlert: false,
    infoAlert: false,
    warningAlert: false,
  });
  const [alertContent, setAlertContent] = useState("");
  const [openGazebo, setOpenGazebo] = useState(false);
  const [openConsole, setOpenConsole] = useState(false);
  const [playState, setPlayState] = useState(true);
  const [brainFreq, setBrainFreq] = useState(brainFreqAck);
  const [frequencyRows, setFrequencyRows] = useState([
    createData("Brain Frequency (Hz)", 0),
    createData("GUI Frequency (Hz)", 0),
    createData("Simulation Real time factor", 0),
  ]);
  let [errorContentHeading, setErrorContentHeading] =
    useState("Errors detected !");
  let [errorContent, setErrorContent] = useState(
    <Typography> No error detected </Typography>
  );
  const [guiFreq, setGuiFreq] = useState(guiFreqAck);

  const [editorCode, setEditorCode] = useState(`from GUI import GUI
from HAL import HAL
# Enter sequential code!

while True:
    # Enter iterative code!`);

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
      const size = get_novnc_size_react();
      ws_manager.send(
        JSON.stringify({
          command: "open",
          exercise: exercise,
          width: size.width.toString(),
          height: size.height.toString(),
        })
      );
      level++;
      connectionUpdate(
        { connection: "exercise", command: "launch_level", level: `${level}` },
        "*"
      );
      ws_manager.send(JSON.stringify({ command: "Pong" }));
      setAlertState({
        ...alertState,
        errorAlert: false,
        successAlert: true,
        warningAlert: false,
        infoAlert: false,
      });
      setAlertContent("Start the Exercise");
    } else if (step === 2) {
      ws_manager.send(JSON.stringify({ command: "exit", exercise: "" }));
    }

    ws_manager.onopen = function (event) {
      level++;
      connectionUpdate({ connection: "manager", command: "up" }, "*");
      connectionUpdate({ connection: "exercise", command: "available" }, "*");
    };

    ws_manager.onclose = function (event) {
      connectionUpdate({ connection: "manager", command: "down" }, "*");
      if (!firstAttempt) {
        setAlertState({
          ...alertState,
          errorAlert: true,
          successAlert: false,
          warningAlert: false,
          infoAlert: false,
        });
        setAlertContent("Connection lost, retrying connection...");
        startSim(step);
      } else {
        firstAttempt = false;
      }
    };

    ws_manager.onerror = function (event) {
      connectionUpdate({ connection: "manager", command: "down" }, "*");
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
          declare_code(address_code);
          declare_gui(address_gui);
        }
        if (gazeboToggle) {
          if (gazeboOn) {
            ws_manager.send(JSON.stringify({ command: "startgz" }));
          } else {
            ws_manager.send(JSON.stringify({ command: "stopgz" }));
          }

          gazeboToggle = false;
        } else if (simStop) {
          ws_manager.send(JSON.stringify({ command: "stop" }));
          simStop = false;
          running = false;
        } else if (simReset) {
          ws_manager.send(JSON.stringify({ command: "reset" }));
          simReset = false;
        } else if (sendCode) {
          let python_code = "#code\n" + editorRef.current.editor.getValue();
          ws_manager.send(
            JSON.stringify({ command: "evaluate", code: python_code })
          );
          sendCode = false;
        } else if (simResume) {
          ws_manager.send(JSON.stringify({ command: "resume" }));
          simResume = false;
          running = true;
        } else {
          setTimeout(function () {
            ws_manager.send(JSON.stringify({ command: "Pong" }));
          }, 1000);
        }
      }
      if (event.data.includes("evaluate")) {
        if (event.data.length < 9) {
          // If there is an error it is sent along with "evaluate"
          submitCode();
        } else {
          let error = event.data.substring(10, event.data.length);
          console.log(error);
          connectionUpdate(
            { connection: "exercise", command: "error", text: error },
            "*"
          );
        }
        setTimeout(function () {
          ws_manager.send(JSON.stringify({ command: "Pong" }));
        }, 1000);
      } else if (event.data.includes("reset")) {
        // Nothing
      } else if (event.data.includes("PingDone")) {
        enableSimControls();
        if (resetRequested === true) {
          resetRequested = false;
        }
      } else if (event.data.includes("style")) {
        let error = event.data.substring(5, event.data.length);
        connectionUpdate(
          { connection: "exercise", command: "style", text: error },
          "*"
        );
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
        // Nothing
      } else if (data.command === "up") {
        stop();
        setLaunchState("Ready");
        togglePlayPause(false);
        let reset_button = document.getElementById("reset");
        reset_button.disabled = false;
        reset_button.style.opacity = "1.0";
        reset_button.style.cursor = "default";
        let load_button = document.getElementById("loadIntoRobot");
        load_button.disabled = false;
        load_button.style.opacity = "1.0";
        load_button.style.cursor = "default";
      } else if (data.command === "down") {
        setLaunchState("Launch");
      } else if (data.command === "launch_level") {
        let level = data.level;
        setLaunchLevel(level);
        setLaunchState("Launching");
      } else if (data.command === "error") {
        console.log(data.text);
        setErrorContent(<Typography>{data.text}</Typography>);
        handleErrorModalOpen();
        toggleSubmitButton(true);
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

  function resetSimulation() {
    simReset = true;
  }

  function stopSimulation() {
    simStop = true;
  }

  function resumeSimulation() {
    simResume = true;
  }

  function checkCode() {
    sendCode = true;
  }

  const declare_gui = (websocket_address) => {
    websocket_gui = new WebSocket(websocket_address);

    websocket_gui.onopen = function (event) {
      if (!resetRequested) setLaunchLevel(launchLevel + 1);
      connectionUpdate(
        { connection: "exercise", command: "launch_level", level: "6" },
        "*"
      );
      if (websocket_code.readyState === 1) {
        if (!resetRequested) {
          setAlertState({
            ...alertState,
            errorAlert: false,
            successAlert: true,
            warningAlert: false,
            infoAlert: false,
          });
          setAlertContent(" Connection established! ");
          connectionUpdate({ connection: "exercise", command: "up" }, "*");
        } else {
          toggleSubmitButton(true);
        }
      }
    };

    websocket_gui.onclose = function (event) {
      // Check for hard reset (reboot exercise.py)
      if (resetRequested && ws_manager.readyState === 1) {
        setTimeout(function () {
          declare_gui(websocket_address);
        }, 500);
      } else {
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
      }
    };

    // What to do when a message from server is received
    websocket_gui.onmessage = function (event) {
      let operation = event.data.substring(0, 4);
      if (operation === "#gui") {
        // Parse the entire Object
        let data = JSON.parse(event.data.substring(4));

        // Parse the Image Data
        let image_data = JSON.parse(data.image),
          source = decode_utf8(image_data.image),
          shape = image_data.shape;

        if (source !== "") {
          image.src = "data:image/jpeg;base64," + source;
          const canvas = guiCanvasRef.current;
          canvas.width = shape[1];
          canvas.height = shape[0];
        }

        // Send the Acknowledgment Message
        websocket_gui.send("#ack");
      }

      if (operation === "#gul") {
        // Parse the entire Object
        let data = JSON.parse(event.data.substring(4));

        // Parse the Image Data
        let image_data = JSON.parse(data.image),
          source = decode_utf8(image_data.image),
          shape = image_data.shape;

        if (source !== "") {
          image_left.src = "data:image/jpeg;base64," + source;
          const canvas_left = guiCanvasRefDrone.current;
          canvas_left.width = shape[1];
          canvas_left.height = shape[0];
        }

        // Send the Acknowledgment Message
        websocket_gui.send("#ack");
      }
    };
  };

  // For image object
  image.onload = function () {
    update_image();
  };

  // For image object
  image_left.onload = function () {
    update_left_image();
  };

  // Request Animation Frame to remove the flickers
  function update_image() {
    window.requestAnimationFrame(update_image);
    const context = guiCanvasRef.current.getContext("2d");
    context.drawImage(image, 0, 0);
  }

  // Request Animation Frame to remove the flickers
  function update_left_image() {
    window.requestAnimationFrame(update_left_image);
    const context_left = guiCanvasRefDrone.current.getContext("2d");
    context_left.drawImage(image_left, 0, 0);
  }

  // WS_CODE_BEGINS

  function declare_code(websocket_address) {
    websocket_code = new WebSocket(websocket_address);

    websocket_code.onopen = function (event) {
      if (!resetRequested)
        connectionUpdate(
          { connection: "exercise", command: "launch_level", level: "5" },
          "*"
        );
      if (websocket_gui.readyState === 1) {
        if (!resetRequested) {
          setAlertState({
            ...alertState,
            errorAlert: false,
            successAlert: true,
            warningAlert: false,
            infoAlert: false,
          });
          setAlertContent(" Connection established! ");
          connectionUpdate({ connection: "exercise", command: "up" }, "*");
        } else {
          toggleSubmitButton(true);
        }
      }
      websocket_code.send("#ping");
    };
    websocket_code.onclose = function (event) {
      // Check for hard reset (reboot exercise.py)
      if (resetRequested && ws_manager.readyState === 1) {
        console.log("retrying...");
        setTimeout(function () {
          declare_code(websocket_address);
        }, 500);
      } else {
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
          setAlertContent(" Connection closed! ");
        }
      }
    };

    websocket_code.onmessage = function (event) {
      let source_code = event.data;
      let operation = source_code.substring(0, 5);

      if (operation === "#load") {
        code = source_code.substring(5);
        setEditorCode(source_code.substring(5));
      } else if (operation === "#freq") {
        let frequency_message = JSON.parse(source_code.substring(5));

        // Parse GUI and Brain frequencies
        const idealGuiFreqValue = frequency_message.gui;
        const idealBrainFreqValue = frequency_message.brain;
        // Parse real time factor
        const rtfValue = frequency_message.rtf;

        setFrequencyRows([
          createData("Brain Frequency (Hz)", idealBrainFreqValue),
          createData("GUI Frequency (Hz)", idealGuiFreqValue),
          createData("Simulation Real time factor", rtfValue),
        ]);

        // The acknowledgement messages invoke the python server to send further
        // messages to this client (inside the server's handle function)
        // Send the acknowledgment message along with frequency
        frequency_message = {
          brain: brainFreqAck,
          gui: guiFreqAck,
          rtf: rtfValue,
        };
        websocket_code.send("#freq" + JSON.stringify(frequency_message));
      } else if (operation === "#ping") {
        websocket_code.send("#ping");
      } else if (operation === "#exec") {
        if (firstCodeSent === false) {
          firstCodeSent = true;
          enablePlayPause(true);
        }
        toggleSubmitButton(true);
      }
    };
  }
  function resumeBrain() {
    let message = "#play\n";
    console.log("Message sent!");
    websocket_code.send(message);
  }

  function stopBrain() {
    let message = "#stop\n";
    console.log("Message sent!");
    websocket_code.send(message);
  }

  function resetBrain() {
    let message = "#rest\n";
    console.log("Message sent!");
    websocket_code.send(message);
  }
  function stopCode() {
    var stop_code = "#code\n";
    console.log("Message sent!");
    websocket_code.send(stop_code);
  }
  function loadCode() {
    // Send message to initiate load message
    let message = "#load";
    websocket_code.send(message);
  }
  // Function that sends/submits the code!
  const submitCode = () => {
    try {
      // Get the code from editor and add headers
      const python_code = "#code\n" + editorRef.current.editor.getValue();
      console.log(python_code);
      websocket_code.send(python_code);
      setAlertState({
        ...alertState,
        errorAlert: false,
        successAlert: false,
        warningAlert: false,
        infoAlert: true,
      });
      setAlertContent(`Code Sent! Check terminal for more information!`);
      // deactivateTeleOpButton();
    } catch {
      setAlertState({
        ...alertState,
        errorAlert: false,
        successAlert: false,
        warningAlert: true,
        infoAlert: false,
      });
      setAlertContent(
        `Connection must be established before sending the code.`
      );
    }
  };
  // WS_CODE_ENDS
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

  const start = () => {
    enablePlayPause(false);
    toggleResetButton(false);
    // Manager Websocket
    if (!running) {
      resumeSimulation();
      resumeBrain();
    }
    togglePlayPause(true);
  };

  // Function to request to load the student code into the robot
  function check() {
    editorChanged(false);
    toggleSubmitButton(false);
    checkCode();
  }
  function editorChanged(toggle) {
    if (firstCodeSent && toggle) {
      setAlertState({
        ...alertState,
        errorAlert: true,
        successAlert: false,
        warningAlert: false,
        infoAlert: false,
      });
      setAlertContent(`Code Changed since last sending`);
    }
  }

  // Function to stop the student solution
  const stop = () => {
    enablePlayPause(false);
    toggleResetButton(false);

    // Manager Websocket
    if (running) {
      stopSimulation();
      stopBrain();
    }

    togglePlayPause(false);
  };

  // Function to reset the simulation
  const resetSim = () => {
    resetRequested = true;
    toggleResetButton(false);
    toggleSubmitButton(false);
    enablePlayPause(false);

    // Manager Websocket
    resetBrain();
    resetSimulation();

    running = false;
  };

  const changeConsole = (openSnackbar) => {
    if (openSnackbar) {
      setAlertState({
        ...alertState,
        errorAlert: false,
        successAlert: false,
        warningAlert: false,
        infoAlert: true,
      });
      setAlertContent(`Console Opened !!`);
    } else {
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

  const changeGzWeb = (openSnackbar) => {
    if (openSnackbar) {
      setAlertState({
        ...alertState,
        errorAlert: false,
        successAlert: false,
        warningAlert: true,
        infoAlert: false,
      });
      setAlertContent(`Gazebo Opened !!`);
    } else {
      setAlertState({
        ...alertState,
        errorAlert: false,
        successAlert: false,
        warningAlert: false,
        infoAlert: true,
      });
      setAlertContent(`Gazebo Closed !!`);
    }
    gazeboOn = !gazeboOn;
    setOpenGazebo(gazeboOn);
    gazeboToggle = true;
  };

  const toggleSubmitButton = (toggle) => {
    var loadIntoRobot = document.getElementById("loadIntoRobot");
    if (toggle === false) {
      loadIntoRobot.disabled = true;
      loadIntoRobot.style.opacity = "0.4";
      loadIntoRobot.style.cursor = "not-allowed";
      handleLoadModalOpen();
    } else {
      loadIntoRobot.disabled = false;
      loadIntoRobot.style.opacity = "1.0";
      loadIntoRobot.style.cursor = "default";
      handleLoadModalClose();
    }
  };

  function toggleResetButton(toggle) {
    let reset_button = document.getElementById("reset");
    if (toggle === false) {
      reset_button.disabled = true;
      reset_button.style.opacity = "0.4";
      reset_button.style.cursor = "not-allowed";
    } else {
      reset_button.disabled = false;
      reset_button.style.opacity = "1.0";
      reset_button.style.cursor = "default";
    }
  }

  function togglePlayPause(stop) {
    setPlayState(!stop);
  }

  function enableSimControls() {
    if (resetRequested === true) {
      togglePlayPause(false);
    }
    enablePlayPause(true);
    toggleResetButton(true);
  }

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

  const loadFileButton = (event) => {
    event.preventDefault();
    let fr = new FileReader();
    fr.onload = (event) => {
      code = fr.result;
      setEditorCode(fr.result);
    };
    fr.readAsText(event.target.files[0]);
  };

  const handleInfoModalOpen = () => setOpenInfoModal(true);
  const handleInfoModalClose = () => setOpenInfoModal(false);
  const handleErrorModalOpen = () => setOpenErrorModal(true);
  const handleErrorModalClose = () => setOpenErrorModal(false);
  const handleLoadModalOpen = () => setOpenLoadModal(true);
  const handleLoadModalClose = () => setOpenLoadModal(false);
  const handleFilenameChange = (event) => setFilename(event.target.value);
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
  const onPageLoad = () => {
    startSim(0);
  };

  const onUnload = () => {
    startSim(2);
    return true;
  };
  const onClickSave = () => {
    saveCode(filename, code);
  };
  const editorCodeChange = (e) => {
    code = e;
    setEditorCode(e);
  };
  return (
    <RescuePeopleExerciseContext.Provider
      value={{
        editorCode,
        openLoadModal,
        connectionState,
        launchState,
        launchLevel,
        openGazebo,
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
        filename,
        editorRef,
        imageRef,
        guiCanvasRef,
        guiCanvasRefDrone,
        handleLoadModalOpen,
        handleLoadModalClose,
        check,
        onClickSave,
        editorCodeChange,
        connectionButtonClick,
        launchButtonClick,
        resetSim,
        start,
        stop,
        loadFileButton,
        changeGzWeb,
        changeConsole,
        handleInfoModalOpen,
        handleInfoModalClose,
        handleErrorModalOpen,
        handleErrorModalClose,
        onPageLoad,
        onUnload,
        handleFilenameChange,
        keyHandleFrequency,
      }}
    >
      {children}
    </RescuePeopleExerciseContext.Provider>
  );
}

ExerciseProvider.propTypes = {
  children: PropTypes.node,
};

export default RescuePeopleExerciseContext;
