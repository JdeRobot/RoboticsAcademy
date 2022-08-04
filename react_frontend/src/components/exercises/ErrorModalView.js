import * as React from "react";
import ExerciseContext from "../../contexts/ExerciseContext";
import Modal from "@mui/material/Modal";
import Box from "@mui/material/Box";
import Typography from "@mui/material/Typography";
import { Divider } from "@mui/material";
const style = {
  position: "absolute",
  top: "50%",
  left: "50%",
  transform: "translate(-50%, -50%)",
  width: 800,
  bgcolor: "background.paper",
  border: "2px solid #000",
  boxShadow: 24,
  p: 4,
};

export default function ErrorModalView() {
  const {
    openErrorModal,
    handleErrorModalClose,
    errorContent,
    errorContentHeading,
  } = React.useContext(ExerciseContext);

  return (
    <div>
      <Modal
        open={openErrorModal}
        onClose={handleErrorModalClose}
        aria-labelledby="modal-modal-title"
        aria-describedby="modal-modal-description"
      >
        <Box id={"control 1"} sx={style}>
          <Typography
            id="modal-modal-title"
            variant="h6"
            component="h2"
            color={"error"}
          >
            {errorContentHeading}
          </Typography>
          <Divider color={"secondary"} variant={"middle"} sx={{ m: 2 }} />
          {errorContent}
        </Box>
      </Modal>
    </div>
  );
}
