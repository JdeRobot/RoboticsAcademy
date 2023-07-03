import * as React from "react";
import PropTypes from "prop-types";
import {  draw } from "./helpers/birds_eye_global_navigation";

function SpecificGlobalNavigation(props) {
  const guiCanvasRef = React.useRef();

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");

    const getMapDataAndDraw = (data) => {
      if (data.map) {
        const pose = data.map.substring(1, data.map.length - 1);
        const content = pose.split(',').map(function(item) {
          return parseFloat(item);
        })
        draw((content[0] ), (content[1] ), content[2], content[3]);
      }
    }
    const getImageAndDisplay = (data) => {
      if(data.image) {
        console.log(data.image)
      let canvas = document.getElementById("gui-canvas-numpy");
        //Parse encoded image data and decode it
      function decode_utf8(s) {
          return decodeURIComponent(escape(s))
      }
			var image_data = JSON.parse(data.image),
			source = decode_utf8(image_data.image),
			shape = image_data.shape;

			if(source !== ""){
				canvas.src = "data:image/png;base64," + source;
				canvas.width = shape[1];
				canvas.height = shape[0];
			}
      }
    }

    const callback = (message) => {
      const data = message.data.update;
      getMapDataAndDraw(data)
      getImageAndDisplay(data)
    };

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

  function destinationPicker(event){
    let mapCanvas = document.getElementById("globalnav-eye");
    let cursorX = (event.clientX - mapCanvas.getBoundingClientRect().left);
    let cursorY = (event.clientY - mapCanvas.getBoundingClientRect().top);
    let cursorXMap = cursorX/mapCanvas.width * 400;
    let cursorYMap = cursorY/mapCanvas.height * 400;
    return [cursorXMap, cursorYMap];
}

  return (
    <div style={{display: "flex",   width: "100%",
    height: "100%"}}>
    <canvas
      ref={guiCanvasRef}
      id="globalnav-eye"
      style={{
        backgroundImage:
          "url('/static/exercises/global_navigation_newmanager/resources/images/cityLargenBin.png')",
          border: "2px solid #d3d3d3",
          backgroundRepeat: "no-repeat",
          backgroundSize: "100% 100%",
          width: "100%",
          height: "100%"
      }}
      width= "400"
      height= "400"
      onClick={function pickLoc(){
        var data = destinationPicker(event)
        console.log(data);
        let coords = {"data" : data};
        try {
          window.RoboticsExerciseComponents.commsManager.send("#pick", data)
        } catch (error) {
          console.error(error)
        }
        
}}
    />
    <img id="gui-canvas-numpy" width="400" height="400" style={{
      	marginTop: "5px",
        width: "100%",
        height: "100%",
        margin: "auto"
    }}></img>
    </div>
  );
}

SpecificGlobalNavigation.propTypes = {
  circuit: PropTypes.string,
};

export default SpecificGlobalNavigation
