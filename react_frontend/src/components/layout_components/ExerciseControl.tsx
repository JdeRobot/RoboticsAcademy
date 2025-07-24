import * as React from "react";
import Toolbar from "@mui/material/Toolbar";
import { Box } from "@mui/material";
import RoboticsTheme from "Components/RoboticsTheme.tsx";
//import PropTypes from "prop-types";
import SaveButton from "Components/buttons/SaveButton.tsx";
import LoadFileButton from "Components/buttons/LoadFileButton.tsx";
import ResetButton from "Components/buttons/ResetButton.tsx";
import Frequencies from "Components/visualizers/Frequencies.tsx";
import PlayPauseButton from "Components/buttons/PlayPauseButton.tsx";
import "../../styles/layout_components/ExerciseControl.css";
import monitor from "../../images/monitoring2.png";
import DLModelUploadButton from "../buttons/DLModelUploadButton.tsx";

interface ExerciseControlProps {
  specificConfiguration?: any;
}

const ExerciseControl: React.FC<ExerciseControlProps> = ({ specificConfiguration }) => {
  const [editorRendered, setEditorRendered] = React.useState<boolean>(false);
  const [showFrequencies, setShowFrequencies] = React.useState<boolean>(false);
  const [buttonActive, setButtonActive] = React.useState<boolean>(false);

  React.useEffect(() => {
    if (document.getElementById("code-container")) {
      setEditorRendered(true);
    }
  }, []);
  
  const handleToggleFrequencies = () => {
    setButtonActive(!buttonActive);
    setShowFrequencies(!showFrequencies);
  };

  return (
    <RoboticsTheme>
      <Toolbar className={"exercise-toolbar"}>
        {editorRendered ? (
          <Box id={"editor-control"}>
            <LoadFileButton />
            <SaveButton />
          </Box>
        ) : null}
        <Box
          id={"robot-control"}
          sx={{
            display: "flex",
            flexWrap: "wrap",
            justifyContent: "space-between",
            alignItems: "center",
            m: 1,
          }}
        >
          <DLModelUploadButton />
          <PlayPauseButton></PlayPauseButton>
          <ResetButton></ResetButton>
          <Frequencies style={showFrequencies ? "visible" : "hidden"} />
          <button
            className={`button ${buttonActive ? "toggledColor" : ""}`}
            onClick={handleToggleFrequencies}
            id="toggleButton"
          >
            <img src={monitor} className="monitor"></img>
          </button>
        </Box>
      </Toolbar>
    </RoboticsTheme>
  );
}

export default ExerciseControl;
