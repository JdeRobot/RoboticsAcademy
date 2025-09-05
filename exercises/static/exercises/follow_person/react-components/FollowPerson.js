import { useState, useEffect } from "react";
import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import noImage from "../../assets/img/noImage.png";
import "./css/GUICanvas.css";

function FollowPerson() {
  const [image, setImage] = useState(noImage);
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  useEffect(() => {
    if (manager === null) {
      return;
    }

    const callback = (message) => {
      const update = message.data.update;
      if (update.image) {
        console.log("New img received");
        const image = JSON.parse(update.image);
        setImage(`data:image/png;base64,${image.image}`);

        // Send the ACK of the img
        manager.send("gui", "ack");
      }
    };

    listen_key();
    manager.subscribe(events.UPDATE, callback);

    return () => {
      manager.unsubscribe(events.UPDATE, callback);
    };
  }, [manager]);

  function listen_key() {
    window.addEventListener("keypress", function (event) {
      if (event.code === "KeyS") {
        manager.send("gui", "key_s");
      } else if (event.code === "KeyW") {
        manager.send("gui", "key_w");
      } else if (event.code === "KeyA") {
        manager.send("gui", "key_a");
      } else if (event.code === "KeyD") {
        manager.send("gui", "key_d");
      } else if (event.code === "KeyX") {
        manager.send("gui", "key_x");
      }
    });
  }

  return (
    <div
      style={{
        display: "flex",
        width: "100%",
        height: "100%",
        position: "relative",
        justifyContent: "center",
      }}
    >
      <img className="image" id="gui_canvas" src={image} />
    </div>
  );
}

export default FollowPerson;
