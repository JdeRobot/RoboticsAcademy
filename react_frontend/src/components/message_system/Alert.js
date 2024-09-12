import * as React from "react";
import MuiAlert from "@mui/material/Alert";
import Button from "@mui/material/Button";
import Collapse from "@mui/material/Collapse";
import Fade from "@mui/material/Fade";

import "../../styles/message_system/Alert.css";

window.RoboticsReactComponents = window.RoboticsReactComponents || {};
window.RoboticsReactComponents.MessageSystem =
    window.RoboticsReactComponents.MessageSystem || {};
window.RoboticsReactComponents.MessageSystem.Alert = (function () {
    let alert_handler = null;

    const setAlertHandler = (callback) => {
        alert_handler = callback;
    };

    /**
     * Create alert component
     * @param message string
     * @param messageType defatult="error", One of the following: ("error", "success", "info", "warning")
     * @param closeAction closeAction
     * @param closeText closeText
     */
    const showAlert = (message, messageType, closeAction, closeText) => {
        if (alert_handler) {
            alert_handler(message, messageType, closeAction, closeText);
        }
    };

    return {
        showAlert: showAlert,
        setAlertHandler: setAlertHandler,
    };
})();

const Alert = () => {
    const [message, setMessage] = React.useState("");
    const [messageType, setMessageType] = React.useState("error");
    const [closeData, setCloseData] = React.useState(null);
    const [show, setShow] = React.useState(true);
    const [alertVisibility, setAlertVisibility] = React.useState(true);
    const timer = 5000;
    const enterSpeed = 1000;
    const exitSpeed = 1000;

    React.useEffect(() => {
        RoboticsReactComponents.MessageSystem.Alert.setAlertHandler(
            (message, messageType, closeAction, closeText) => {
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
                setMessageType(messageType || "error");

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
