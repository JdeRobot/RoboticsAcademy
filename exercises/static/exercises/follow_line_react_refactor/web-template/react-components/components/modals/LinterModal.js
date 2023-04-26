import React from "react";
import { Box, Modal, Typography } from "@mui/material";

const LinterModal = (props) => {
  const [linterMessage, setLinterMessage] = React.useState(props.message);

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

  return (<div id={"modal-container"}>
    {
      linterMessage ?
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
        : null
    }
    </div>
    )
};

LinterModal.propTypes = {

};

LinterModal.SetLinterMessage = (message) => {
  setLinterMessage(message);
};


export default LinterModal;