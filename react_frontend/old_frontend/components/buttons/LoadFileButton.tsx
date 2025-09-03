import CloudUploadOutlinedIcon from "@mui/icons-material/CloudUploadOutlined";
import { Button } from "@mui/material";
import * as React from "react";
//import PropTypes from "prop-types";

declare global {
  interface Window {
    RoboticsReactComponents: any;
  }
}

interface LoadFileButtonProps {
  context?: any;
}

const LoadFileButton: React.FC<LoadFileButtonProps> = () => {
  const loadFile = (event: React.ChangeEvent<HTMLInputElement>) => {
    event.preventDefault();
    const fr = new FileReader();
    fr.onload = () => {
      window.RoboticsReactComponents.CodeEditor.setCode(fr.result);
    };
    fr.readAsText(event.target.files?.[0]!);
  };
  return (
    <Button
      variant="contained"
      sx={{ m: 1 }}
      color={"secondary"}
      startIcon={<CloudUploadOutlinedIcon />}
      component="label"
    >
      Load file
      <input hidden accept=".py" type="file" onChange={loadFile} />
    </Button>
  );
};

export default LoadFileButton;
