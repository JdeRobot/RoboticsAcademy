import { useState, useEffect } from "react";
import noImage from "../../assets/img/noImage.png";
import { useExercise } from "Contexts/ExerciseContext";
import { events } from "jderobot-commsmanager";

import "./css/GUICanvas.css";

function WebGUI() {
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
      if (message.data.update.image) {
        var canvas = document.getElementById("gui_canvas");

        // Request Animation Frame to remove the flickers
        function decode_utf8(s) {
          return decodeURIComponent(escape(s));
        }

        // Parse the Image Data
        var image_data = JSON.parse(message.data.update.image),
          source = decode_utf8(image_data.image),
          shape = image_data.shape;

        if (source != "" && shape instanceof Array) {
          canvas.src = "data:image/jpeg;base64," + source;
          canvas.width = shape[1];
          canvas.height = shape[0];
        }
      }

      // Send the ACK of the msg
      manager.send("gui", "ack");
    };

    manager.subscribe([events.UPDATE], callback);

    return () => {
      manager.unsubscribe([events.UPDATE], callback);
    };
  }, [manager]);

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
      <img className="image" id="gui_canvas" src={noImage} />
    </div>
  );
}

export default WebGUI;
