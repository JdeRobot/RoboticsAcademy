import {Fragment, useState} from "react";

import "./css/TestLoader.css"
import classNames from "classnames";

const TestLoader = (props) => {
    const [state, setState] = useState("idle");
    const [waiting, setWaiting] = useState(false);

    const classes = classNames({
        "test-loader": true,
        "disabled": waiting
    });

    const spinnerClasses = classNames({
        "lds-ring": true,
        "hidden": !waiting
    })

    const buttonClick = (event) => {
        if (state === 'idle') {
            doConnect();
        } else if (state === 'connected') {
            doLaunch();
        } else if (state === 'ready') {
            doTerminate();
        }
    };

    const doConnect = () => {
        setWaiting(true);

        RoboticsExerciseComponents.commsManager.connect()
            .then((message) => {
                setState('connected');
            }).catch((response) => {

            }).finally(() => {
                setWaiting(false);
            });
    }

    const doLaunch = () => {
        setWaiting(true);
        const config = JSON.parse(document.getElementById("exercise-config").textContent);

        // Setting up circuit name into configuration
        config.application.params = { circuit: "default" };
        let launch_file = config.launch['0'].launch_file.interpolate({ circuit: 'default' });
        config.launch['0'].launch_file = launch_file;

        RoboticsExerciseComponents.commsManager.launch(config)
            .then((message) => {
                setState('ready');
            }).catch((response) => {

            }).finally(() => {
                setWaiting(false);
            });
    }

    const doTerminate = () => {
        setWaiting(true);

        RoboticsExerciseComponents.commsManager.terminate()
            .then((message) => {
                setState('connected');
            }).catch((response) => {

            }).finally(() => {
                setWaiting(false)
            });
    }

    const buttonText = () => {
        if (state === 'idle') {
            return "Connect";
        } else if (state === 'connected') {
            return "Launch";
        } else if (state === 'ready') {
            return "Terminate";
        }
    }

    return (
        <div className={'parent-block'}>
            <div className={classes} onClick={buttonClick}>{buttonText()}</div>
            <div className={spinnerClasses}>
                <div></div>
                <div></div>
                <div></div>
                <div></div>
            </div>
        </div>
    );
}

export default TestLoader;