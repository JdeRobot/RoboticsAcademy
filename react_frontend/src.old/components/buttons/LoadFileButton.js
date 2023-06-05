import CloudUploadOutlinedIcon from "@mui/icons-material/CloudUploadOutlined";
import { Button } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";

export const LoadFileButton = (props) => {
  const { loadFileButton } = React.useContext(props.context);
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

LoadFileButton.propTypes = {
  context: PropTypes.any,
};
