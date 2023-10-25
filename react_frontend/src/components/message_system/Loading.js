import * as React from "react";
import {
  Button,
  Box,
  CircularProgress,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
} from "@mui/material";

import "../../styles/message_system/Loading.css";

window.RoboticsReactComponents = window.RoboticsReactComponents || {};
window.RoboticsReactComponents.MessageSystem =
  window.RoboticsReactComponents.MessageSystem || {};
window.RoboticsReactComponents.MessageSystem.Loading = (function () {
  let handler = null;
  let failHandler = null;

  const showLoading = (message, show = true) => {
    if (handler) {
      handler(message, show);
    }
  };

  const showFailLoading = (message, action, show = true) => {
    if (failHandler) {
      failHandler(message, action, show);
    }
  };

  const hideLoading = () => {
    showLoading("", false);
  };

  const hideFailLoading = () => {
    showFailLoading("", null, false);
  };

  const subscribeHandler = (handlerFunc) => {
    handler = handlerFunc;
  };

  const subscribeFailHandler = (failHandlerFunc) => {
    failHandler = failHandlerFunc;
  };

  return {
    showLoading: showLoading,
    hideLoading: hideLoading,
    showFailLoading: showFailLoading,
    hideFailLoading: hideFailLoading,
    subscribeHandler: subscribeHandler,
    subscribeFailHandler: subscribeFailHandler,
  };
})();

const Loading = () => {
  const [open, setOpen] = React.useState(false);
  const [message, setMessage] = React.useState("");
  const [closeData, setCloseData] = React.useState(null);

  React.useEffect(() => {
    RoboticsReactComponents.MessageSystem.Loading.subscribeHandler(
      (message, show) => {
        setMessage(message);
        if (show) {
          handleOpen();
        } else {
          handleClose();
        }
      }
    );
  }, []);

  React.useEffect(() => {
    RoboticsReactComponents.MessageSystem.Loading.subscribeFailHandler(
      (message, action, show) => {
        setMessage(message);
        if (show) {
          handleOpen();
        } else {
          handleClose();
        }
        if (action) {
          setCloseData(action);
        }
      }
    );
  }, []);

  const handleClose = () => {
    setOpen(false);
    setCloseData(null);
  };

  const handleOpen = () => {
    setOpen(true);
  };

  const handle = () => {
    window.location.reload();
  };

  const openInstructions = () => {
    window.open("/instructions/", "_blank");
  };

  return (
    <Dialog open={open} onClose={handleClose} className="fixed-size-dialog">
      <DialogContent className="center-content">
        <Box
          sx={{
            display: "flex",
            flexDirection: "column",
            alignItems: "center",
            justifyContent: "center",
          }}
        >
          <DialogTitle id="responsive-dialog-title">{message}</DialogTitle>
          {!closeData && <CircularProgress />}
        </Box>
      </DialogContent>
      {closeData ? (
        <DialogActions>
          <Button onClick={openInstructions}>RADI Instructions</Button>
          <Button onClick={handle}>Reload</Button>
        </DialogActions>
      ) : (
        ""
      )}
    </Dialog>
  );
};

export default Loading;
