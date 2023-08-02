import React, { useEffect } from "react";
import { useState } from "react";
import "../../styles/Indicator.css";

export const ConnectionIndicator = () => {
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "connected") {
        setConnected(true);
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
      <span className="word">Robotics</span>
      <span className="word">Backend</span>
    </div>
  );
};
