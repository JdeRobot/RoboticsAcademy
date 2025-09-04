import { useState, useEffect } from "react";
import { events } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import "./css/GUICanvas.css";
import Car from "../resources/images/car-top-view.svg";

const Autoparking = () => {
  const meter = 73; // 1m = 73px

  const [laser, setLaser] = useState([]);
  const [maxRange, setMaxRange] = useState([]);
  const [laser1, setLaser1] = useState([]);
  const [maxRange1, setMaxRange1] = useState([]);
  const [laser2, setLaser2] = useState([]);
  const [maxRange2, setMaxRange2] = useState([]);
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
      if (message.data.update.map) {
        const map_data = JSON.parse(message.data.update.map);
        console.log(map_data);
        setLaser(map_data.lasers[0]);
        setLaser1(map_data.lasers[1]);
        setLaser2(map_data.lasers[2]);
        setMaxRange(map_data.ranges[0]);
        setMaxRange1(map_data.ranges[1]);
        setMaxRange2(map_data.ranges[2]);
        console.log(map_data.ranges);
      }
      // Send the ACK of the msg
      manager.send("gui", "ack");
    };

    manager.subscribe(events.UPDATE, callback);

    return () => {
      manager.unsubscribe(events.UPDATE, callback);
    };
  }, [manager]);

  return (
    <div
      style={{
        display: "flex",
        width: "100%",
        height: "100%",
        backgroundColor: "#363233",
        position: "relative",
        overflow: "hidden",
      }}
    >
      <img src={Car} id="car" />
      {laser.map((element) => {
        var ang = -element[1];
        var length = (element[0] / 75) * meter;
        return (
          <hr
            className="laser-beam"
            style={{
              rotate: "z " + ang + "rad",
              width: length + "px",
              position: "absolute",
              background:
                "repeating-linear-gradient(to right,rgb(255, 112, 112),rgb(255, 112, 112) 73px,rgb(175, 29, 29)  73px,rgb(175, 29, 29) 146px)",
              backgroundSize: "100% 1px",
              bottom: "59%",
              left: "50%",
              transformOrigin: "0% 0%",
              zIndex: "3",
            }}
          />
        );
      })}
      {laser1.map((element) => {
        var ang = -element[1] + Math.PI / 2;
        var length = (element[0] / 75) * meter;
        return (
          <hr
            className="laser-beam"
            style={{
              rotate: "z " + ang + "rad",
              width: length + "px",
              position: "absolute",
              background:
                "repeating-linear-gradient(to right,rgb(112, 138, 255),rgb(112, 138, 255) 73px,rgb(100, 198, 255) 73px,rgb(100, 198, 255) 146px)",
              backgroundSize: "100% 1px",
              bottom: "50%",
              left: "52%",
              transformOrigin: "0% 0%",
              zIndex: "4",
            }}
          />
        );
      })}
      {laser2.map((element) => {
        var ang = -element[1] + Math.PI;
        var length = (element[0] / 75) * meter;
        return (
          <hr
            className="laser-beam"
            style={{
              rotate: "z " + ang + "rad",
              width: length + "px",
              position: "absolute",
              background:
                "repeating-linear-gradient(to right,rgb(112, 255, 119),rgb(112, 255, 119) 73px,rgb(18, 138, 14) 73px,rgb(18, 138, 14) 146px)",
              backgroundSize: "100% 1px",
              bottom: "41%",
              left: "50%",
              transformOrigin: "0% 0%",
              zIndex: "3",
            }}
          />
        );
      })}
    </div>
  );
};

export default Autoparking;
