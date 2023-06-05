/* eslint-disable no-unused-vars */
import * as React from "react";
import { createContext, useRef, useState } from "react";
import PropTypes from "prop-types";
import { get_novnc_size_react, saveCode } from "../helpers/utils";
import { Typography } from "@mui/material";
import { configMap } from "../helpers/AutoParking/MapView";

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
  swapping = false,
  gazeboOn = false,
  gazeboToggle = false;
let code = `from GUI import GUI
from HAL import HAL
# Enter sequential code!

while True:
    # Enter iterative code!`;
let openConsoleSnackbar = true;
let openGazeboSnackbar = true;
function createData(key, value) {
  return { key, value };
}

const AutoParkingExerciseContext = createContext();

export function ExerciseProvider({ children }) {
  const exercise = "autoparking";
  const exerciseSpecificCSS = "local-map-lasers";
  const [filename, setFilename] = useState("Auto parking");
  const editorRef = useRef();
  const guiCanvasRef = useRef();
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
    let level = 0;
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
          let python_code = editorRef.current.editor.getValue();
          python_code = "#code\n" + python_code;
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
        // launchButton.prop("disabled", false);
      } else if (data.command === "down") {
        setConnectionState("Connect");
        if (websocket_code != null) websocket_code.close();
        if (websocket_gui != null) websocket_gui.close();
        setLaunchState("Launch");
      }
    } else if (data.connection === "exercise") {
      if (data.command === "available") {
        // Nothing !
      } else if (data.command === "up") {
        stop();
        swapping = false;
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
        if (!swapping) {
          setLaunchState("Launch");
        }
      } else if (data.command === "swap") {
        setLaunchState("Launching");
      } else if (data.command === "launch_level") {
        const level = data.level;
        setLaunchLevel(level);
        setLaunchState("Launching");
      } else if (data.command === "error") {
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
        //
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

  function declare_code(websocket_address) {
    websocket_code = new WebSocket(websocket_address);

    websocket_code.onopen = function (event) {
      connectionUpdate(
        { connection: "exercise", command: "launch_level", level: "5" },
        "*"
      );
      if (websocket_gui.readyState === 1) {
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
      websocket_code.send("#ping");
    };
    websocket_code.onclose = function (event) {
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
    };

    websocket_code.onmessage = function (event) {
      const source_code = event.data;
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
        let code_frequency = brainFreqAck;
        let gui_frequency = guiFreqAck;
        frequency_message = {
          brain: code_frequency,
          gui: gui_frequency,
          rtf: rtfValue,
        };
        frequency_message = { brain: code_frequency, gui: gui_frequency };
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
  // Function that sends/submits the code!
  const submitCode = () => {
    try {
      // Get the code from editor and add headers
      const python_code = "#code\n" + editorRef.current.editor.getValue();
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
  // Function that send/submits an empty string
  const stopCode = () => {
    const stop_code = "#code\n";
    websocket_code.send(stop_code);
  };

  // Function for range slider
  const codeFrequencyUpdate = (e) => {
    brainFreqAck = brainFreqAck + 1;
    setBrainFreq(brainFreqAck);
  };

  // Function for range slider
  const guiFrequencyUpdate = (e) => {
    guiFreqAck = guiFreqAck + 1;
    setGuiFreq(guiFreqAck);
  };

  // WS GUI BEGINS
  function declare_gui(websocket_address) {
    websocket_gui = new WebSocket(websocket_address);

    websocket_gui.onopen = function (event) {
      connectionUpdate(
        { connection: "exercise", command: "launch_level", level: "6" },
        "*"
      );
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
      const operation = event.data.substring(0, 4);
      if (operation === "#gui") {
        // Parse the entire Object
        const data = JSON.parse(event.data.substring(4));

        // Parse the Map data
        const map_data = JSON.parse(data.map);
        configMap(
          guiCanvasRef.current,
          map_data.car,
          map_data.obstacle,
          map_data.average,
          map_data.lasers,
          map_data.ranges
        );

        // Send the Acknowledgement Message
        websocket_gui.send("#ack");
      }
    };
  }

  // WS GUI ENDS

  // Function to resume the simulation
  function start() {
    enablePlayPause(false);
    toggleResetButton(false);
    // Manager Websocket
    if (running === false) {
      resumeSimulation();
      resumeBrain();
    }
    togglePlayPause(true);
  }

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
  function stop() {
    enablePlayPause(false);
    toggleResetButton(false);

    // Manager Websocket
    if (running === true) {
      stopSimulation();
      stopBrain();
    }

    togglePlayPause(false);
  }

  // Function to reset the simulation
  function resetSim() {
    resetRequested = true;
    toggleResetButton(false);
    enablePlayPause(false);

    // Manager Websocket
    resetBrain();
    resetSimulation();

    running = false;
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

  const changeGzWeb = () => {
    if (openGazeboSnackbar) {
      openGazeboSnackbar = false;
      setAlertState({
        ...alertState,
        errorAlert: false,
        successAlert: false,
        warningAlert: true,
        infoAlert: false,
      });
      setAlertContent(`Gazebo Opened !!`);
    } else {
      openGazeboSnackbar = true;
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

  const loadFileButton = (event) => {
    event.preventDefault();
    const fr = new FileReader();
    fr.onload = (event) => {
      code = fr.result;
      setEditorCode(fr.result);
    };
    fr.readAsText(event.target.files[0]);
  };
  const onClickSave = () => {
    saveCode(filename, code);
  };

  const handleInfoModalOpen = () => setOpenInfoModal(true);
  const handleInfoModalClose = () => setOpenInfoModal(false);
  const handleErrorModalOpen = () => setOpenErrorModal(true);
  const handleErrorModalClose = () => setOpenErrorModal(false);
  const handleLoadModalOpen = () => setOpenLoadModal(true);
  const handleLoadModalClose = () => setOpenLoadModal(false);
  const handleFilenameChange = (event) => setFilename(event.target.value);
  const editorCodeChange = (e) => {
    code = e;
    setEditorCode(e);
  };
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

  const onPageLoad = () => {
    startSim(0);
  };

  const onUnload = () => {
    startSim(2);
    return true;
  };

  return (
    <AutoParkingExerciseContext.Provider
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
        exerciseSpecificCSS,
        editorRef,
        guiCanvasRef,
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
        startSim,
        connectionUpdate,
        changeGzWeb,
        changeConsole,
        codeFrequencyUpdate,
        guiFrequencyUpdate,
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
    </AutoParkingExerciseContext.Provider>
  );
}

ExerciseProvider.propTypes = {
  children: PropTypes.node,
};

export default AutoParkingExerciseContext;
