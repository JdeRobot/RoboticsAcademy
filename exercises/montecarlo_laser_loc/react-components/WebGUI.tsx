import { useState, useEffect, useRef } from "react";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import houseMap from "./resources/mapgrannyannie.png";
import Vacuum from "./resources/vacuum.svg";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import "./css/GUICanvas.css";

const WebGUI = () => {
  const exerciseContext = useExercise();
  const [vacuumPose, setVacuumPose] = useState<number[] | undefined>(undefined);
  const [userPose, setUserPose] = useState<number[] | undefined>(undefined);
  const [userParticles, setParticles] = useState<number[][]>([]);
  const canvasRef = useRef<HTMLImageElement>(null);
  const [manager, setManager] = useState(exerciseContext.manager);
  const vacuumSize = 40;
  var lastRealPose: number[] | undefined = undefined;
  var lastUserPose: number[] | undefined = undefined;

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const resizeObserver = new ResizeObserver((entries) => {
    var img = entries[0].target;
    //or however you get a handle to the IMG
    var width = img.clientWidth / 300;
    var height = img.clientHeight / 150;

    if (lastRealPose) {
      setVacuumPose([
        lastRealPose[1] * height - (vacuumSize * img.clientHeight) / 1012,
        lastRealPose[0] * width - (vacuumSize * img.clientWidth) / 1012,
        -lastRealPose[2],
      ]);
    }

    if (lastUserPose) {
      setUserPose([
        lastUserPose[1] * height - (vacuumSize * img.clientHeight) / 1012,
        lastUserPose[0] * width - (vacuumSize * img.clientWidth) / 1012,
        -lastUserPose[2],
      ]);
    }

    setParticles([]);
  });

  const updateCallback = (updateData: unknown) => {
    let data = updateData as any;
    const update = data.update;

    var img = canvasRef.current;
    if (img === null) {
      return;
    }
    var width = img.clientWidth / 300;
    var height = img.clientHeight / 150;

    if (update.map) {
      const pose = update.map.substring(1, update.map.length - 1);
      const content = pose.split(",").map((item: string) => parseFloat(item));
      const poseUser = update.user.substring(1, update.user.length - 1);
      const userContent = poseUser
        .split(",")
        .map((item: string) => parseFloat(item));

      lastRealPose = content;

      setVacuumPose([
        content[1] * height - (vacuumSize * img.clientHeight) / 1012,
        content[0] * width - (vacuumSize * img.clientWidth) / 1012,
        -content[2],
      ]);

      if (
        !(userContent[0] === 0 && userContent[1] === 0 && userContent[2] === 0)
      ) {
        lastUserPose = userContent;
        setUserPose([
          userContent[1] * height - (vacuumSize * img.clientHeight) / 1012,
          userContent[0] * width - (vacuumSize * img.clientWidth) / 1012,
          -userContent[2],
        ]);
      }
    }

    if (update.particles) {
      const particles = JSON.parse(update.particles);
      if (particles != "") {
        var new_particles: number[][] = [];
        particles.forEach((element: number[]) => {
          new_particles.push([
            element[1] * height,
            element[0] * width,
            -element[2],
            element[3],
          ]);
        });
        setParticles(new_particles);
      }
    }
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setVacuumPose(undefined);
      setUserPose(undefined);
      setParticles([]);
      lastRealPose = undefined;
      lastUserPose = undefined;
    }
  };

  connectApplication(
    manager,
    updateCallback,
    stateCallback,
    canvasRef,
    resizeObserver
  );

  return (
    <WebGUIContainer>
      <WebGUIImage
        reference={canvasRef}
        id="map-img"
        src={houseMap}
        style={{ left: "0", width: "100%" }}
      />
      <div className="overlay" id="map-container">
        {vacuumPose && (
          <div
            className="vacuum"
            style={{
              rotate: "z " + vacuumPose[2] + "rad",
              top: vacuumPose[0],
              height: (vacuumSize * canvasRef.current.clientHeight) / 1012,
              left: vacuumPose[1],
              width: (vacuumSize * canvasRef.current.clientWidth) / 1012,
            }}
          >
            <img src={Vacuum} />
            <div
              className="arrow"
              style={{
                height: (vacuumSize * canvasRef.current.clientHeight) / 1012,
                marginLeft:
                  -((vacuumSize * canvasRef.current.clientWidth) / 1012) / 2,
              }}
            />
          </div>
        )}
        {userPose && (
          <div
            className="vacuum"
            style={{
              rotate: "z " + userPose[2] + "rad",
              top: userPose[0],
              height: (vacuumSize * canvasRef.current.clientHeight) / 1012,
              left: userPose[1],
              width: (vacuumSize * canvasRef.current.clientWidth) / 1012,
            }}
          >
            <img src={Vacuum} id="user-pos" />
            <div
              className="arrow arrow-user"
              style={{
                height: (vacuumSize * canvasRef.current.clientHeight) / 1012,
                marginLeft:
                  -((vacuumSize * canvasRef.current.clientWidth) / 1012) / 2,
              }}
            />
          </div>
        )}
        {userParticles.map((element) => {
          return (
            <div
              className="particle"
              style={{
                rotate: "z " + element[2] + "rad",
                top: element[0] - 5,
                left: element[1] - 5,
                opacity: element[3],
              }}
            >
              <div className="particle-arrow" />
            </div>
          );
        })}
      </div>
    </WebGUIContainer>
  );
};

export default WebGUI;
