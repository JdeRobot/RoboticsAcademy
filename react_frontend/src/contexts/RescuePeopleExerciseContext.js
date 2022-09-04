/* eslint-disable no-unused-vars */
import * as React from "react";
import { createContext, useState, useRef } from "react";
import PropTypes from "prop-types";
import { Typography } from "@mui/material";
import { saveCode } from "../helpers/utils";
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
  gazeboToggle = false,
  is_local = false;
let operation, data;
let image_data, source, shape;
let code = `from GUI import GUI
from HAL import HAL
# Enter sequential code!

while True:
    # Enter iterative code!`;

function createData(key, value) {
  return { key, value };
}

const GlobalNavigationExerciseContext = createContext();

export function ExerciseProvider({ children }) {
  const exercise = "obstacle_avoidance";
  const editorRef = useRef();
  const imageRef = useRef();
  const localMapRef = useRef();
  // connectionState - Connect, Connecting, Connected
  const [connectionState, setConnectionState] = useState("Connect");
  // launchState - Launch, Launching, Ready
  const [launchState, setLaunchState] = useState("Launch");
  const [launchLevel, setLaunchLevel] = useState(0);
  const [openInfoModal, setOpenInfoModal] = useState(false);
  const [openErrorModal, setOpenErrorModal] = useState(false);
  const [openLoadModal, setOpenLoadModal] = useState(false);
  const [filename, setFilename] = useState("obstacle-avoidance");
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
    var fr = new FileReader();
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
    <GlobalNavigationExerciseContext.Provider
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
        localMapRef,
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
    </GlobalNavigationExerciseContext.Provider>
  );
}

ExerciseProvider.propTypes = {
  children: PropTypes.node,
};

export default GlobalNavigationExerciseContext;
