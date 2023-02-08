import CloudUploadOutlinedIcon from "@mui/icons-material/CloudUploadOutlined";
import { Button } from "@mui/material";
import * as React from "react";

export const LoadFileButton = () => {
  return (
    <Button
      variant="contained"
      sx={{ m: 1 }}
      color={"secondary"}
      startIcon={<CloudUploadOutlinedIcon />}
      component="label"
    >
      Load file
      <input hidden accept=".py" type="file" />
    </Button>
  );
};
