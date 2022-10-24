import * as React from 'react';
import classNames from 'classnames';

import './css/TestExerciseManager3.css';
import {Fragment} from "react";

const Test3ExerciseManager = () => {
  const [state, setState] = React.useState("Idle");

  const classes = classNames({
    "test-exercise-manager-state": true,
  });

  React.useEffect(() => {
    console.log("Test3ExerciseManager subscribing to ['state-change'] events");

    const callback = (message) => {
          setState(message.data.state);
          console.log(message);
    };

    RoboticsExerciseComponents.commsManager.subscribe([
          RoboticsExerciseComponents.commsManager.events.STATE_CHANGED
        ],
        callback);

    return () => {
      console.log("Test2ExerciseManager unsubscribing from ['state-changed'] events");
      RoboticsExerciseComponents.commsManager.unsubscribe([
            RoboticsExerciseComponents.commsManager.events.STATE_CHANGED
          ],
        callback);
    }
  }, []);

  return (
    <Fragment>
        <hr />
        <div className={classes}>CURRENT STATE: {state}</div>
    </Fragment>
  )
}

export default Test3ExerciseManager;