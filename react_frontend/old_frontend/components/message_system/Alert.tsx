import * as React from "react";
import MuiAlert from "@mui/material/Alert";
import Button from "@mui/material/Button";
import Collapse from "@mui/material/Collapse";
import Fade from "@mui/material/Fade";

import "../../styles/message_system/Alert.css";

declare global {
  interface Window {
    RoboticsReactComponents: {
      MessageSystem: {
        Alert: {
          showAlert: (
            message: string | string[],
            messageType?: string,
            closeAction?: () => void,
            closeText?: string
          ) => void;
          setAlertHandler: (
            callback: (
              message: string | string[],
              messageType?: string,
              closeAction?: () => void,
              closeText?: string
            ) => void
          ) => void;
        };
      };
    };
  }
}

window.RoboticsReactComponents = window.RoboticsReactComponents || {};
window.RoboticsReactComponents.MessageSystem =
    window.RoboticsReactComponents.MessageSystem || {};


window.RoboticsReactComponents.MessageSystem.Alert =
  window.RoboticsReactComponents.MessageSystem.Alert || (function () {
    let alert_handler: ((...args: any[]) => void) | null = null;

    const setAlertHandler = (callback: (...args: any[]) => void) => {
      alert_handler = callback;
    };

    /**
     * Create alert component
     * @param message string
     * @param messageType defatult="error", One of the following: ("error", "success", "info", "warning")
     * @param closeAction closeAction
     * @param closeText closeText
     */
    const showAlert = (
      message: string | string[],
      messageType: string = "error",
      closeAction?: () => void,
      closeText?: string
    ) => {
      if (alert_handler) {
        alert_handler(message, messageType, closeAction, closeText);
      }
    };

    return {
        showAlert: showAlert,
        setAlertHandler: setAlertHandler,
    };
})();

const Alert: React.FC = () => {
    const [message, setMessage] = React.useState<React.ReactNode>(null);
    const [messageType, setMessageType] = React.useState<string>("error");
    const [closeData, setCloseData] = React.useState<{
    text: string;
    action: () => void;
    } | null>(null);
    const [show, setShow] = React.useState<boolean>(true);
    const [alertVisibility, setAlertVisibility] = React.useState<boolean>(false);
    
    const timer = 5000;
    const enterSpeed = 1000;
    const exitSpeed = 1000;

    React.useEffect(() => {
        RoboticsReactComponents.MessageSystem.Alert.setAlertHandler(
            (
            message: string | string[],
            type: string = "error",
            closeAction?: () => void,
            closeText?: string
            ) => {
                setAlertVisibility(true)
                if (Array.isArray(message)) {
                    message = message.map((msg, i) => <p key={i}>{msg}</p>);
                } else if (typeof message !== "string") {
                    console.error(`Bad message sent ${message}`);
                    return;
                } else {
                    message = <p>{message}</p>;
                }

                setMessage(message || "No message set");
                setMessageType(type || "error");

                if (closeAction && closeText) {
                    setCloseData({
                        text: closeText,
                        action: closeAction,
                    });
                }

                // const timeId = setTimeout(() => {
                //     setShow(false)
                // }, timer)
                //
                // return () => {
                //     clearTimeout(timeId)
                // }
            }
        );

    }, []);

    const closeAlert = () => {
        setMessage(null);
        setMessageType(null);
        setCloseData(null);
    };



    if (!show) {
        return false;
    } else {
        return (
            <Fade
                in={alertVisibility} //Write the needed condition here to make it appear
                timeout={{
                    enter: enterSpeed,
                    exit: exitSpeed
                }} // Edit these two values to change the duration of transition when the element is getting appeared and disappeard
                addEndListener={() => {
                    setTimeout(() => {
                        setAlertVisibility(false)
                        // closeAlert()
                    }, timer);
                }}
            >
                <div id={"message-container"} className={"bottom"}>
                    {message ? (
                        <Collapse in={message !== null}>
                            <MuiAlert
                                severity={messageType ? messageType : "error"}
                                action={
                                    <Button
                                        color="inherit"
                                        size="small"
                                        onClick={closeData ? closeData.action : closeAlert}
                                    >
                                        {closeData ? closeData.text : "Close"}
                                    </Button>
                                }
                            >
                                <div className="message-scroll">{message}</div>
                            </MuiAlert>
                        </Collapse>
                    ) : null}
                </div>
            </Fade>
        );
    }

};

export default Alert;
