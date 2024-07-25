import * as React from "react";
import PropTypes from "prop-types";
import {drawImage, startStreaming} from "./helpers/showImagesColorFilter";
// The stream & capture
//var stream = document.getElementById('stream');


function SpecificColorFilter(props) {
  const [image, setImage] = React.useState(null)
  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");
    // Start Streaming
    //startStreaming()
    const callback = (message) => {
      console.log(message);

      if (message.data.update.image) {
        drawImage(message.data.update);
      }

      // Send the ACK of the msg
      window.RoboticsExerciseComponents.commsManager.send("gui", "ack");
    };

   /*return () => {
        
   // Start Streaming
    startStreaming()
    };*/
   
    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    return () => {
      console.log("TestShowScreen unsubscribing from ['state-changed'] events");
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
        callback
      );
    };
  }, []);



  return (
    <div style={{display: "flex",   width: "100%",
    height: "100%"}}>
    <canvas id="gui_canvas"></canvas>
    </div>
  );
}


SpecificColorFilter.propTypes = {
  circuit: PropTypes.string,
};

/*setTimeout(function(){
    console.log("START LAUNCHER");
    //startStreaming();
    UseCamera();
    //declare_webrtcframe();
}, 15000);*/

export default SpecificColorFilter
