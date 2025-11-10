import { useState, useEffect, useRef} from "react";
import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import houseMap from "../resources/images/GUI_mapgrannyannie.png";
import Vacuum from "../resources/images/vacuum.svg";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer from "Components/exercise/WebGUIContainer";
import "./css/GUICanvas.css";

function WebGUI(props) {
  const [vacuumPose, setVacuumPose] = useState(null)
  const [userPose, setUserPose] = useState(null)
  const [userParticles, setParticles] = useState([])
  const canvasRef = useRef(null);
  const [userImage, setUserImage] = useState(undefined);
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  var lastRealPose = undefined;
  var lastUserPose = undefined;

  const resizeObserver = new ResizeObserver((entries) => {
    var img = entries[0].target; 
    //or however you get a handle to the IMG
    var width = img.clientWidth / 300;
    var height = img.clientHeight / 150;

    if (lastRealPose) {
      setVacuumPose([lastRealPose[1]*height,lastRealPose[0]*width, -lastRealPose[2]]);
    }

    if (lastUserPose) {
      setUserPose([lastUserPose[1]*height,lastUserPose[0]*width, -lastUserPose[2]]);
    }

    setParticles([])
  });

  useEffect(() => {
    if (manager === null) {
      return;
    }

    const updateCallback = (message) => {
      const updateData = message.data.update;
      
      // Lógica para manejar la imágen
      if (updateData.image) {
        let image = JSON.parse(updateData.image);
        if (image.shape instanceof Array) {
          setUserImage(`data:image/png;base64,${image.image}`);
        }
      }
      
      // Lógica para manejar el mapa
      var img = canvasRef.current 
      var width = img.clientWidth / 300;
      var height = img.clientHeight / 150;

      if (updateData.map) {
        const pose = updateData.map.substring(1, updateData.map.length - 1);
        const content = pose.split(",").map(item => parseFloat(item));
        const poseUser = updateData.user.substring(1, updateData.user.length - 1);
        const userContent = poseUser.split(",").map(item => parseFloat(item));

        lastRealPose = content;

        setVacuumPose([content[1]*height,content[0]*width, -content[2]]);

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
            new_particles.push([element[1]*height, element[0]*width, -element[2], element[3]])
          });
          setParticles(new_particles)
        }
      }

      // Send the ACK of the msg
      manager.send("gui", "ack");
    };

    const stateCallback = (message) => {
      if (message.data.state === "tools_ready") {
        setVacuumPose(null)
        setUserPose(null)
        setParticles([])
        lastRealPose = undefined;
        lastUserPose = undefined;
        setUserImage()
      }
    }

    var img = canvasRef.current
    if (img) {
      resizeObserver.observe(img);
    }

    manager.subscribe(events.UPDATE, updateCallback);
    manager.subscribe(events.STATE_CHANGED, stateCallback);

    return () => {
      manager.unsubscribe(events.UPDATE, updateCallback);
      manager.unsubscribe(events.STATE_CHANGED, stateCallback);
    };
  }, [manager]);

  return (
  <WebGUIContainer>
    <WebGUIImage reference={canvasRef} id="map-img" src={houseMap} style={{ left: "0" }} />
      <div className="overlay" id="map-container">
        {vacuumPose &&
          <div id="vacuum-pos" style={{rotate: "z "+ vacuumPose[2]+"rad", top: vacuumPose[0] -10 , left: vacuumPose[1] -1}}>
            <img src={Vacuum} id="vacuum-pos"/>
            <div className="arrow arrow-real"/>
          </div>
        }
        {userPose &&
          <div id="user-pos" style={{rotate: "z "+ userPose[2]+"rad", top: userPose[0] -10 , left: userPose[1] -10}}>
            <img src={Vacuum} id="user-pos"/>
            <div className="arrow arrow-user"/>
          </div>
        }
        {userParticles.map(element => {
            return (
              <div className="particle" style={{rotate: "z "+ element[2]+"rad", top: element[0] -5, left: element[1] -5, opacity: element[3]}}>
                <div className="particle-arrow"/>
              </div>
          )})
        }
      </div>
      <WebGUIImage src={userImage} id="map-img" style={{ left: "50%" }} />
    </WebGUIContainer>
  );
}

export default WebGUI
