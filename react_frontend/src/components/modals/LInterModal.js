import { Box, Modal, Typography } from "@mui/material";
import React, { useContext } from "react";
import PropTypes from "prop-types";

export const LinterModal = (props) => {
  const { linterMessage, setLinterMessage } = useContext(props.context);

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

  return (
    <Modal
      open={!!linterMessage.length}
      onClose={() => {
        setLinterMessage([]);
      }}
      aria-labelledby="modal-modal-title"
      aria-describedby="modal-modal-description"
    >
      <Box sx={style}>
        {linterMessage.map((line) => (
          <Typography key={line}>{line}</Typography>
        ))}
      </Box>
    </Modal>
  );
};

LinterModal.propTypes = {
  context: PropTypes.any,
};
