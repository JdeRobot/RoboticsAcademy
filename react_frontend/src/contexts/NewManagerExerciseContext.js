import * as React from "react";
import PropTypes from "prop-types";
import CommsManager from "../libs/comms_manager";
import { useState } from "react";
import { saveCode } from "../helpers/utils";
const NewManagerExerciseContext = React.createContext();

export function ExerciseProvider({ children }) {
  const ramHost = window.location.hostname;
  const ramPort = 7163;
  CommsManager(`ws://${ramHost}:${ramPort}`);

  const [alertState, setAlertState] = useState({
    errorAlert: false,
    successAlert: false,
    infoAlert: false,
    warningAlert: false,
  });
  const [launchLevel, setLaunchLevel] = useState(0);
  const [alertContent, setAlertContent] = useState("");
  const [openGazebo, setOpenGazebo] = useState(false);
  const [openConsole, setOpenConsole] = useState(false);

  // connectionState - Connect, Connecting, Connected
  const [connectionState, setConnectionState] = useState("Connect");

  // launchState - Launch, Launching, Ready
  const [launchState, setLaunchState] = useState("Launch");
  const createData = (key, value) => {
    return { key, value };
  };
  const [frequencyRows, setFrequencyRows] = useState([
    createData("Brain Frequency (Hz)", 0),
    createData("GUI Frequency (Hz)", 0),
    createData("Simulation Real time factor", 0),
  ]);
  const [editorCode, setEditorCode] = useState(`from GUI import GUI
from HAL import HAL
# Enter sequential code!

while True:
    # Enter iterative code!`);

  const [filename, setFileName] = useState("filename");

  const [playState, setPlayState] = useState(false);

  const startSim = async () => {
    if (connectionState === "Connect") {
      await RoboticsExerciseComponents.commsManager.connect().then(() => {
        setConnectionState("Connected");
      });
    }
  };
  const connectionButtonClick = () => {
    if (connectionState === "Connect") {
      setConnectionState("Connecting");
      startSim();
    }
  };

  const getConfig = () => {
    return JSON.parse(document.getElementById("exercise-config").textContent);
  };

  const doLaunch = async (config) => {
    await RoboticsExerciseComponents.commsManager
      .launch(config)
      .then((message) => {
        setLaunchState("Ready");
        console.log(message);
      })
      .catch((response) => {
        console.log(response, "response");
        setLaunchState("Launch");
      })
      .finally(() => {});
  };

  const launchButtonClick = () => {
    if (connectionState === "Connected" && launchState === "Launch") {
      const config = getConfig();
      setLaunchState("Launching");
      doLaunch(config);
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

  const terminate = async () => {
    await RoboticsExerciseComponents.commsManager
      .terminate()
      .then(() => {
        console.log("terminated");
        setLaunchState("Launching");
      })
      .catch((response) => {
        console.log(response, "error terminating");
      });
  };

  const editorCodeChange = (e) => {
    setEditorCode(e);
  };

  const onPageLoad = () => {
    console.log("onPageLoad");
  };
  const onUnload = () => {
    console.log("onUnload");
  };

  const submitCode = () => {
    try {
      // Get the code from editor and add headers
      console.log({ code: editorCode });
      RoboticsExerciseComponents.commsManager
        .send("load", {
          code: editorCode,
        })
        .then((message) => {
          console.log(message);
        })
        .catch((response) => {
          console.error(response);
        });
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

  const loadFileButton = (event) => {
    event.preventDefault();
    var fr = new FileReader();
    fr.onload = () => {
      setEditorCode(fr.result);
    };
    fr.readAsText(event.target.files[0]);
  };

  const saveFileButton = () => {
    saveCode(filename, editorCode);
  };

  const resetSim = () => {
    RoboticsExerciseComponents.commsManager
      .reset()
      .then(() => {
        console.log("reseting");
      })
      .catch((response) => console.log(response));
  };

  const handleFilename = (e) => {
    setFileName(e.target.value);
  };

  const changeGzWeb = () => {
    console.log("gazebo");
    if (!openGazebo) {
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
    setOpenGazebo(!openGazebo);
  };

  const changeConsole = () => {
    if (!openConsole) {
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

  const stop = () => {
    RoboticsExerciseComponents.commsManager.pause().then(() => {
      setPlayState(false);
      console.log("STOPED");
    });
  };

  const start = () => {
    RoboticsExerciseComponents.commsManager.run().then(() => {
      setPlayState(true);
      console.log("PLAYING");
    });
  };

  const [openInfoModal, setOpenInfoModal] = useState(false);
  const handleInfoModalOpen = () => setOpenInfoModal(true);
  const [openLoadModal, setOpenLoadModal] = useState(false);
  const handleLoadModalClose = () => setOpenLoadModal(false);

  return (
    <NewManagerExerciseContext.Provider
      value={{
        onPageLoad,
        onUnload,
        startSim,
        connectionButtonClick,
        launchButtonClick,
        handleLoadModalClose,
        handleInfoModalOpen,
        openInfoModal,
        openLoadModal,
        alertState,
        alertContent,
        connectionState,
        launchState,
        launchLevel,
        frequencyRows,
        setFrequencyRows,
        submitCode,
        editorCodeChange,
        editorCode,
        resetSim,
        loadFileButton,
        saveFileButton,
        handleFilename,
        changeGzWeb,
        openGazebo,
        changeConsole,
        openConsole,
        stop,
        start,
        playState,
        terminate,
        doLaunch,
      }}
    >
      {children}
    </NewManagerExerciseContext.Provider>
  );
}

ExerciseProvider.propTypes = {
  children: PropTypes.node,
};

export default NewManagerExerciseContext;
