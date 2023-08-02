import React, { useEffect } from "react";
import { useState } from "react";
import "../../styles/Indicator.css";

export const ApplicationIndicator = () => {
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const callback = (message) => {
      setConnected(message.data.state);
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

  return (
    <div className={connected === "running" ? "ready" : "waiting"}>
      <span className="word">Application</span>
      <span className="word">Running</span>
    </div>
  );
};
