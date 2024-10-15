import * as React from "react";
import PropTypes from "prop-types";
import houseMap from "../resources/images/mapgrannyannie.png";
import Vacuum from "../resources/images/vacuum.svg";

import "./css/GUICanvas.css";

function SpecificMontecarloLaserLoc(props) {
  const [vacuumPose, setVacuumPose] = React.useState(null)
  const [userPose, setUserPose] = React.useState(null)
  const [userParticles, setParticles] = React.useState([])

  var lastRealPose = undefined;
  var lastUserPose = undefined;

  const resizeObserver = new ResizeObserver((entries) => {
    var img = entries[0].target; 
    //or however you get a handle to the IMG
    var width = (1013 / 300) / (1013 /img.clientWidth);
    var height = (1012 / 150) / (1012 /img.clientHeight);

    if (lastRealPose) {
      setVacuumPose([lastRealPose[1]*height,lastRealPose[0]*width, -lastRealPose[2]]);
    }

    if (lastUserPose) {
      setUserPose([lastUserPose[1]*height,lastUserPose[0]*width, -lastUserPose[2]]);
    }

    setParticles([])
  });

  React.useEffect(() => {
    console.log("TestShowScreen subscribing to ['update'] events");

    const callback = (message) => {
      const updateData = message.data.update;
      // Lógica para manejar el mapa
      var img = document.getElementById('exercise-img'); 
      var width = (1012 / 300) / (1012 /img.clientWidth);
      var height = (1012 / 150) / (1012 /img.clientHeight);

      if (updateData.map) {
        const pose = updateData.map.substring(1, updateData.map.length - 1);
        const content = pose.split(",").map(item => parseFloat(item));
        const poseUser = updateData.user.substring(1, updateData.user.length - 1);
        const userContent = poseUser.split(",").map(item => parseFloat(item));

        lastRealPose = content;

        setVacuumPose([content[1]*height,content[0]*width, -content[2]]);
        console.log(userContent)

        if (!(userContent[0] === 0 && userContent[1] === 0 && userContent[2] === 0)) {
          lastUserPose = userContent;
          setUserPose([userContent[1]*height,userContent[0]*width, -userContent[2]]);
        }
      }

      if (updateData.particles){
        const particles = JSON.parse(updateData.particles);
        if(particles != "") {
          var new_particles = [];
          particles.forEach(element => {
            new_particles.push([element[1]*height, element[0]*width, -element[2]])
          });
          setParticles(new_particles)
        }
      }

      // Send the ACK of the msg
      window.RoboticsExerciseComponents.commsManager.send("gui", "ack");
    };

    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    resizeObserver.observe(document.getElementById('exercise-img'));

    return () => {
      console.log("TestShowScreen unsubscribing from ['state-changed'] events");
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
        callback
      );
    };
  }, []);

  React.useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "ready") {
        try {
          setVacuumPose(null)
          setUserPose(null)
        } catch (error) {
        }
      }
    }
    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
      callback
    );

    return () => {
      console.log("TestShowScreen unsubscribing from ['state-changed'] events");
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
        callback
      );
    };
  }, [])

  return (
    <div style={{display: "flex", width: "100%", height: "100%", position:"relative"}}>
      <img src={houseMap} alt="" className="exercise-canvas" id="exercise-img"/>
      <div className="overlay" id="map-container">
        {vacuumPose &&
          <div id="vacuum-pos" style={{rotate: "z "+ vacuumPose[2]+"rad", top: vacuumPose[0] -15 , left: vacuumPose[1] -15}}>
            <img src={Vacuum} id="vacuum-pos"/>
            <div className="arrow arrow-real"/>
          </div>
        }
        {userPose &&
          <div id="user-pos" style={{rotate: "z "+ userPose[2]+"rad", top: userPose[0] -15 , left: userPose[1] -15}}>
            <img src={Vacuum} id="user-pos"/>
            <div className="arrow arrow-user"/>
          </div>
        }
        {userParticles.map(element => {
            return (
              <div className="particle" style={{rotate: "z "+ element[2]+"rad", top: element[0] -5, left: element[1] -5}}>
                <div className="particle-arrow"/>
              </div>
          )})
        }
      </div>
    </div>
  );
}


SpecificMontecarloLaserLoc.propTypes = {
  circuit: PropTypes.string,
};

export default SpecificMontecarloLaserLoc
