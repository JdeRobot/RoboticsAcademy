import * as React from 'react';
import classNames from 'classnames';

import './css/TestExerciseManager2.css';

const Test2ExerciseManager = () => {
  const [message, setMessage] = React.useState("");
  const [iserror, setIsError] = React.useState(false);

  const classes = classNames({
    "test-exercise-manager-message": true,
    "error": iserror
  });

  React.useEffect(() => {
    console.log("Test2ExerciseManager subscribing to ['ack','error'] events");
    const callback = (message) => {
          setIsError(message.command==='error');
          setMessage(message.data.message);
          console.log(message.data.message);
    };

    RoboticsExerciseComponents.commsManager.subscribe(['ack', 'error'],
        callback);

    return () => {
      console.log("Test2ExerciseManager unsubscribing from ['ack','error'] events");
      RoboticsExerciseComponents.commsManager.unsubscribe(['ack', 'error'],
        callback);
    }
  }, []);

  return (
    <div className={classes}>{message}</div>
  )
}

export default Test2ExerciseManager;