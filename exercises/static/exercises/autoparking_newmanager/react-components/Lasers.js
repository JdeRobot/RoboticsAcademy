import React from "react"
import "./css/GUICanvas.css";
import Car from "../resources/images/car-top-view.svg";

const Lasers = (props) => {
    const [laser, setLaser] = React.useState([])
    const [maxRange, setMaxRange] = React.useState([])
    const [laser1, setLaser1] = React.useState([])
    const [maxRange1, setMaxRange1] = React.useState([])
    const [laser2, setLaser2] = React.useState([])
    const [maxRange2, setMaxRange2] = React.useState([])

    React.useEffect(() => {
        const callback = (message) => {
            if(message.data.update.map){
              const map_data = JSON.parse(message.data.update.map);
              setLaser (map_data.lasers[0])
              setLaser1(map_data.lasers[1])
              setLaser2(map_data.lasers[2])
              setMaxRange (map_data.ranges[0])
              setMaxRange1(map_data.ranges[1])
              setMaxRange2(map_data.ranges[2])
              console.log(map_data.ranges)
            }
            // Send the ACK of the msg
            window.RoboticsExerciseComponents.commsManager.send("gui", "ack");
        };
        RoboticsExerciseComponents.commsManager.subscribe(
          [RoboticsExerciseComponents.commsManager.events.UPDATE],
          callback
        );
    
        return () => {
          console.log("TestShowScreen unsubscribing from ['state-changed'] events");
          RoboticsExerciseComponents.commsManager.unsubscribe(
            [RoboticsExerciseComponents.commsManager.events.UPDATE],
            callback
          );
        };
      }, []);

    return (
      <div style={{display: "flex",   width: "100%",
        height: "100%", backgroundColor: "#363233", position:"relative", overflow:"hidden"
      }}>
        <img src={Car} id="car"/>
        {laser.map(element => {
          var ang = -element[1]
          var length = (element[0] / maxRange)*20;
          return (
            <hr className="laser-beam" 
              style={{
              rotate: "z "+ ang +"rad",
              width: "calc("+length + "%)",
              position: "absolute",
              background: "repeating-linear-gradient(to right,rgb(255, 112, 112),rgb(255, 112, 112) 73px,rgb(175, 29, 29)  73px,rgb(175, 29, 29) 146px)",
              backgroundSize: "100% 1px",
              bottom: "55%",
              left: "50%",
              transformOrigin: "0% 0%",
              zIndex: "3"}}
            />
        )})
        }
        {laser1.map(element => {
          var ang = -element[1] + Math.PI/2
          var length = (element[0] / maxRange1)*20;
          return (
            <hr className="laser-beam" 
              style={{
              rotate: "z "+ ang +"rad",
              width: "calc("+length + "%)",
              position: "absolute",
              background: "repeating-linear-gradient(to right,rgb(112, 138, 255),rgb(112, 138, 255) 73px,rgb(100, 198, 255) 73px,rgb(100, 198, 255) 146px)",
              backgroundSize: "100% 1px",
              bottom: "50%",
              left: "50%",
              transformOrigin: "0% 0%",
              zIndex: "4"}}
            />
          )})
        }
        {laser2.map(element => {
          var ang = -element[1] + Math.PI
          var length = (element[0] / maxRange2)*20;
          return (
            <hr className="laser-beam" 
              style={{
              rotate: "z "+ ang +"rad",
              width: "calc("+length + "%)",
              position: "absolute",
              background: "repeating-linear-gradient(to right,rgb(112, 255, 119),rgb(112, 255, 119) 73px,rgb(18, 138, 14) 73px,rgb(18, 138, 14) 146px)",
              backgroundSize: "100% 1px",
              bottom: "45%",
              left: "50%",
              transformOrigin: "0% 0%",
              zIndex: "3"}}
            />
          )})
        }
	    </div>
)};

export default Lasers;