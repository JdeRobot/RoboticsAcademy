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

  const click = (command) => {
    RoboticsExerciseComponents.commsManager.launch()
    .then((message) => {
      setMessage("Ready");
      setIserror(false);
    }).catch((response) => {
      setMessage(`Message not received, reason: ${response.data.message}`);
      setIserror(true);
    })
  };

  React.useEffect(() => {
    RoboticsExerciseComponents.commsManager.connect()
      .then(() => {
        setMessage('Connected');
        setIserror(false);
      })
      .catch(() => {
        setMessage('Connection error');
        setIserror(true);
      });
  }, []);

  return (
      <Fragment>
        <div className={"test-exercise-manager"} onClick={click}>Launch</div>
        <div className={"test-exercise-manager"} onClick={
          () => RoboticsExerciseComponents.commsManager.send('fail')
        }>Bad message</div>
        <div className={classes}>{message}</div>
      </Fragment>
  );
};

export default TestExerciseManager;