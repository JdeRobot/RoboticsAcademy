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

  const [openGazebo, setOpenGazebo] = useState(false);
  const [openConsole, setOpenConsole] = useState(false);
  const [filename, setFileName] = useState("filename");
  const [editorCode, setEditorCode] = useState(`from GUI import GUI
from HAL import HAL
# Enter sequential code!

while True:
    # Enter iterative code!`);

  const editorCodeChange = (e) => {
    setEditorCode(e);
  };

  const submitCode = async () => {
    await window.RoboticsExerciseComponents.commsManager
      .send("load", {
        code: editorCode,
      })
      .then((message) => {
        console.log(message);
      })
      .catch((response) => {
        console.error(response);
      });
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

  const handleFilename = (e) => {
    setFileName(e.target.value);
  };

  const changeGzWeb = () => {
    setOpenGazebo(!openGazebo);
  };

  const changeConsole = () => {
    setOpenConsole(!openConsole);
  };

  return (
    <NewManagerExerciseContext.Provider
      value={{
        submitCode,
        editorCodeChange,
        editorCode,
        loadFileButton,
        saveFileButton,
        handleFilename,
        changeGzWeb,
        openGazebo,
        changeConsole,
        openConsole,
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
