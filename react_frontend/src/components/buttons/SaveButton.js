import * as React from "react";
import SaveIcon from "@mui/icons-material/Save";
import { Button } from "@mui/material";

export const SaveButton = () => {
  return (
    <Button
      id={"save"}
      variant="contained"
      color={"secondary"}
      startIcon={<SaveIcon />}
      sx={{ m: 1 }}
    >
      Save file
    </Button>
  );
};
