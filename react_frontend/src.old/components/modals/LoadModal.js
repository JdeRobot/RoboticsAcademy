import * as React from "react";
import Box from "@mui/material/Box";
import Button from "@mui/material/Button";
import Typography from "@mui/material/Typography";
import Modal from "@mui/material/Modal";
import FollowLineExerciseContext from "../../contexts/FollowLineExerciseContext";
import PropTypes from "prop-types";

const style = {
  position: "absolute",
  top: "50%",
  left: "50%",
  transform: "translate(-50%, -50%)",
  width: 400,
  bgcolor: "background.paper",
  border: "2px solid #000",
  boxShadow: 24,
  p: 4,
};

export default function LoadModal(props) {
  const { openLoadModal, handleLoadModalClose } = React.useContext(
    props.context
  );
  return (
    <div>
      <Modal
        open={openLoadModal}
        onClose={handleLoadModalClose}
        aria-labelledby="modal-modal-title"
        aria-describedby="modal-modal-description"
      >
        <Box sx={style}>
          <Typography id="modal-modal-title" variant="h6" component="h2">
            Processing...
          </Typography>
          <Typography id="modal-modal-description" sx={{ mt: 2 }}>
            Wait some seconds. Your code is been loading into the robot...
          </Typography>
        </Box>
      </Modal>
    </div>
  );
}

LoadModal.propTypes = {
  context: PropTypes.any,
};
