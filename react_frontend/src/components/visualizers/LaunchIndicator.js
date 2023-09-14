import React, { useEffect } from "react";
import { useState } from "react";
import "../../styles/Indicator.css";

export const LaunchIndicator = () => {
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const callback = (message) => {
      if (
        (message.data.state === "ready") |
        (message.data.state === "paused") |
        (message.data.state === "running")
      ) {
        setConnected(true);
      } else {
        setConnected(false);
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

  return (
    <div className={connected ? "ready" : "waiting"}>
      <span className="word">World</span>
      <span className="word">Launched</span>
    </div>
  );
};
