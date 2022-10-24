import * as React from 'react';
import {Fragment, useEffect, useState} from 'react';
import * as log from 'loglevel';
import classNames from 'classnames';

import './css/TestExerciseManager.css';

const TestExerciseManager = (props) => {
  const [message, setMessage] = useState("");
  const [iserror, setIserror] = useState(false);
  const classes = classNames({
    "test-exercise-manager-message": true,
    "error": iserror
  });

  const launch = (command) => {
    const config = JSON.parse(document.getElementById("exercise-config").textContent);

    RoboticsExerciseComponents.commsManager.launch(config)
    .then((message) => {
      setMessage("Ready");
      setIserror(false);
    }).catch((response) => {
      setMessage(`Message not received, reason: ${response.data.message}`);
      setIserror(true);
    })
  };

  const connect = (command) => {
    RoboticsExerciseComponents.commsManager.connect()
    .then((message) => {
      setMessage("Connected");
      setIserror(false);
    }).catch((response) => {
      setMessage(`Message not received, reason: ${response.data.message}`);
      setIserror(true);
    })
  };

  return (
      <Fragment>
        <div className={"test-exercise-manager"} onClick={connect}>Connect</div>
        <div className={"test-exercise-manager"} onClick={launch}>Launch</div>
        
        <div className={classes}>{message}</div>
      </Fragment>
  );
};

export default TestExerciseManager;