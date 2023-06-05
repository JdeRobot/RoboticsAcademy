import * as React from "react";
import SaveIcon from "@mui/icons-material/Save";
import { Button } from "@mui/material";
import { saveCode } from "../../helpers/utils";
import PropTypes from "prop-types";

const SaveFileButton = (props) => {
  const saveFile = () => {
    const userCode = RoboticsReactComponents.CodeEditor.getCode();
    saveCode("test", userCode);
  };
  return (
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
  );
};

SaveFileButton.propTypes = {
  context: PropTypes.any,
};

export default SaveFileButton;
