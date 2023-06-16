import * as React from "react";
import SaveIcon from "@mui/icons-material/Save";
import { Box, Button, TextField } from "@mui/material";
import { saveCode } from "../../helpers/utils";
import PropTypes from "prop-types";

const SaveFileButton = (props) => {
  const [fileName, setFileName] = React.useState("myCode");
  const saveFile = () => {
    const userCode = RoboticsReactComponents.CodeEditor.getCode();
    saveCode(fileName, userCode);
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
        onChange={(e) => {
          setFileName(e.target.value);
        }}
      />
    </Box>
  );
};

SaveFileButton.propTypes = {
  context: PropTypes.any,
};

export default SaveFileButton;
