import CloudUploadOutlinedIcon from "@mui/icons-material/CloudUploadOutlined";
import { Button } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";

export const LoadFileButton = (props) => {
  const loadFileButton = (event) => {
    event.preventDefault();
    var fr = new FileReader();
    fr.onload = () => {
      RoboticsReactComponents.CodeEditor.setCode(fr.result);
    };
    fr.readAsText(event.target.files[0]);
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
      <input hidden accept=".py" type="file" onChange={loadFileButton} />
    </Button>
  );
};