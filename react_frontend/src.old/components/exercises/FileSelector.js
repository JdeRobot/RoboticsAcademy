import * as React from "react";
import PropTypes from "prop-types";
import { Box, Button, Typography } from "@mui/material";
import CloudUploadOutlinedIcon from "@mui/icons-material/CloudUploadOutlined";

const FileSelector = (props) => {
  const { fileRef, loadModelButton, FileLabel, acceptExtensions } =
    React.useContext(props.context);

  return (
    <Box
      sx={{
        m: 1,
        p: 2,
        flexGrow: 1,
        width: "100%",
        flexDirection: "column",
        border: "2px solid",
      }}
      id="code-control"
    >
      <Typography> {FileLabel}</Typography>
      <Button
        variant="contained"
        sx={{ m: 1 }}
        color={"secondary"}
        startIcon={<CloudUploadOutlinedIcon />}
        component="label"
      >
        Choose file
        <input
          hidden
          accept={acceptExtensions}
          type="file"
          ref={fileRef}
          onChange={loadModelButton}
        />
      </Button>
    </Box>
  );
};

FileSelector.propTypes = {
  context: PropTypes.any,
};

export default FileSelector;
