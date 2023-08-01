import Button from '@mui/material/Button';
import { useEffect, useState } from 'react';

export const ResetCar = () => {
  const [disabled, setDisabled] = useState(true)
  useEffect(() => {
    const callback = (message) => {
      const state = message.data.state;
      if (state === "running" || state === "paused"){
        setDisabled(false)
      }

    };

    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
      callback
    );

    return () => {
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
        callback
      );
    };
  }, []);
    const handleClick = () => {
        window.RoboticsExerciseComponents.commsManager
          .send("#gui", {
            msg: "#rst",
          })
    }
    return (
        <Button variant="outlined" onClick={handleClick} sx={{color: "blue", borderColor: "blue"}} disabled={disabled}>Reset Car</Button>
    )
}