import * as React from "react";
import {Fragment} from "react";
import SaveIcon from "@mui/icons-material/Save";
import {Button, TextField} from "@mui/material";
import {saveCode} from "Helpers/utils";
import PropTypes from "prop-types";
import LinterModal from "../modals/LinterModal";

export const SaveButton = (props) => {
  const [filename, setFileName] = React.useState("filename");

  const saveCode = (event) => {
    const code = RoboticsReactComponents.CodeEditor.getCode();
    console.log(`Saving code:\n${code}`);
  };

  return (
    <Fragment>
      <Button
        id={"save"}
        variant="contained"
        color={"secondary"}
        startIcon={<SaveIcon/>}
        sx={{m: 1}}
        onClick={saveCode}
      >
        Save file
      </Button>
      <TextField
        sx={{m: "6px"}}
        size={"small"}
        id="filename"
        label="Filename"
        color={"secondary"}
        value={filename}
        onChange={(e) => {
          setFileName(e.target.value);
        }}
      />
    </Fragment>
  )
};

SaveButton.propTypes = {
  context: PropTypes.any,
};
