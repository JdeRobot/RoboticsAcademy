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

  const [visualization, setVisualization] = useState({
    specific: true,
    gazebo: false,
    console: false,
  });
  const [filename, setFileName] = useState("filename");
  const [editorCode, setEditorCode] = useState(`from GUI import GUI
from HAL import HAL
# Enter sequential code!

while True:
    # Enter iterative code!`);
  const [linterMessage, setLinterMessage] = useState([]);
  const editorCodeChange = (e) => {
    setEditorCode(e);
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

  const changeVisualization = (visual) => {
    setVisualization(visual);
  };

  return (
    <NewManagerExerciseContext.Provider
      value={{
        editorCodeChange,
        filename,
        setFileName,
        editorCode,
        loadFileButton,
        saveFileButton,
        handleFilename,
        visualization,
        changeVisualization,
        linterMessage,
        setLinterMessage,
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
