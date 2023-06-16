import * as React from "react";
import { CircularProgress, Backdrop } from "@mui/material";
import Typography from "@mui/material/Typography";

import "../../styles/message_system/Loading.css";

window.RoboticsReactComponents = window.RoboticsReactComponents || {};
window.RoboticsReactComponents.MessageSystem =
  window.RoboticsReactComponents.MessageSystem || {};
window.RoboticsReactComponents.MessageSystem.Loading = (function () {
  let handler = null;

  const showLoading = (message, show = true) => {
    if (handler) {
      handler(message, show);
    }
  };

  const hideLoading = () => {
    showLoading("", false);
  };

  const subscribeHandler = (handlerFunc) => {
    handler = handlerFunc;
  };

  return {
    showLoading: showLoading,
    hideLoading: hideLoading,
    subscribeHandler: subscribeHandler,
  };
})();

const Loading = () => {
  const [open, setOpen] = React.useState(false);
  const [message, setMessage] = React.useState("");

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

  const handleClose = () => {
    setOpen(false);
  };

  const handleOpen = () => {
    setOpen(true);
  };

  return (
    <Backdrop open={open}>
      <div className={"backdrop-message"}>{message}</div>
      <CircularProgress />
    </Backdrop>
  );
};

export default Loading;
