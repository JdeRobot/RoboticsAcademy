import * as React from 'react';
import classNames from 'classnames';

import './css/TestExerciseManager3.css';

const Test3ExerciseManager = () => {
  const [state, setState] = React.useState("");

  const classes = classNames({
    "test-exercise-manager-state": true,
  });

  React.useEffect(() => {
    console.log("Test3ExerciseManager subscribing to ['state-change'] events");
    const callback = (message) => {
          setMessage(message.data.message);
          console.log(message.data.message);
    };

    RoboticsExerciseComponents.commsManager.subscribe(['state-change'],
        callback);

    return () => {
      console.log("Test2ExerciseManager unsubscribing from ['state-change'] events");
      RoboticsExerciseComponents.commsManager.unsubscribe(['state-change'],
        callback);
    }
  }, []);

  return (
    <div className={classes}>CURRENT STATE: {state}</div>
  )
}

export default Test3ExerciseManager;