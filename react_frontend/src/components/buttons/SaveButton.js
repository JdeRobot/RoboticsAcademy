import * as React from "react";
import SaveIcon from "@mui/icons-material/Save";
import { Button } from "@mui/material";
import { saveCode } from "../../helpers/utils";
import PropTypes from "prop-types";

export const SaveButton = (props) => {
  const { editorCode, filename } = React.useContext(props.context);
  return (
    <Button
      id={"save"}
      variant="contained"
      color={"secondary"}
      startIcon={<SaveIcon />}
      sx={{ m: 1 }}
      onClick={() => {
        saveCode(filename, editorCode);
      }}
    >
      Save file
    </Button>
  );
};

SaveButton.propTypes = {
  context: PropTypes.any,
};
