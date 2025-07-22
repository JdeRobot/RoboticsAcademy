import * as React from "react";
import { ChangeEvent, useState } from "react";
import SaveIcon from "@mui/icons-material/Save";
import { Box, Button, TextField } from "@mui/material";
import { saveCode } from "../../helpers/utils";
import PropTypes from "prop-types";

declare global {
  interface Window {
    RoboticsReactComponents: any;
  }
}

const SaveFileButton: React.FC = () => {
  const [fileName, setFileName] = React.useState<string>("myCode");
  
  const saveFile = (): void => {
    let userCode: string = window.RoboticsReactComponents.CodeEditor.getCode();
    saveCode(fileName, userCode);
  };

  const handleChange = (e: ChangeEvent<HTMLInputElement>): void => {
    setFileName(e.target.value);
  };
  
  return (
    <Box sx={{ display: "flex" }}>
      <Button
        id={"save"}
        variant="contained"
        color={"secondary"}
        startIcon={<SaveIcon />}
        sx={{ m: 1 }}
        onClick={saveFile}
      >
        Save file
      </Button>
      <TextField
        sx={{ m: 1 }}
        size={"small"}
        id="filename"
        label="Filename"
        color={"secondary"}
        value={fileName}
        onChange={handleChange}
      />
    </Box>
  );
};

export default SaveFileButton;
