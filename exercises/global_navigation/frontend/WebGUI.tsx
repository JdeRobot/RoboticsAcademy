import React, { useState, useEffect, useRef } from "react";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import { updatePath } from "./helpers/GlobalNavigationHelper";
import Car from "./resources/car-top-view.svg";
import CityMap from "./resources/cityLargenBin.png";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import "./css/GUICanvas.css";

let coords: number[] | undefined = undefined;

function WebGUI() {
  const exerciseContext = useExercise();
  const [carPose, setCarPose] = useState<number[] | undefined>(undefined);
  const [destination, setDestination] = useState<number[] | undefined>(
    undefined
  );
  const [path, setPath] = useState<string>("");
  const [userImage, setUserImage] = useState<string | undefined>(undefined);
  const [manager, setManager] = useState(exerciseContext.manager);
  const canvasRef = useRef<HTMLImageElement>(null);
  let trail: number[][] = [];
  let lastPose: number[] | undefined = undefined;
  let showMap: boolean = false;

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const resizeObserver = new ResizeObserver((entries) => {
    const img = entries[0].target;
    const width = img.clientWidth / 400;
    const height = img.clientHeight / 400;

    updatePath(trail, setPath, height, width);

    if (lastPose) {
      setCarPose([
        lastPose[1] * height,
        lastPose[0] * width,
        Math.PI - lastPose[2],
      ]);
    }
  });

  const getMapDataAndDraw = (data: any) => {
    if (data.map && showMap) {
      const pose = data.map.substring(1, data.map.length - 1);
      const content = pose.split(",").map((item: string) => parseFloat(item));
      lastPose = content;

      const img = canvasRef.current;
      if (img === null) {
        return;
      }
      const width = img.clientWidth / 400;
      const height = img.clientHeight / 400;

      setCarPose([
        content[1] * height,
        content[0] * width,
        Math.PI - content[2],
      ]);
    }
  };

  const getImageAndDisplay = (data: any) => {
    if (data.image) {
      const image = JSON.parse(data.image);
      if (image.image != "" && image.shape instanceof Array) {
        setUserImage(`data:image/png;base64,${image.image}`);
      }
    }
  };

  const getPathAndDisplay = (data: any) => {
    if (data.array && showMap) {
      const img = canvasRef.current;
      if (img === null) {
        return;
      }
      const width = img.clientWidth / 400;
      const height = img.clientHeight / 400;

      trail = JSON.parse(data.array);
      updatePath(trail, setPath, height, width);
    }
  };

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;
    const update = data.update;
    getMapDataAndDraw(update);
    getImageAndDisplay(update);
    getPathAndDisplay(update);
  };

  function destinationPicker(event: any) {
    const img = canvasRef.current;
    if (img === null) {
      return;
    }
    const rect = img.getBoundingClientRect();

    //or however you get a handle to the IMG
    const width = img.clientWidth / 400;
    const height = img.clientHeight / 400;

    const cursorX = event.clientX - rect.left;
    const cursorY = event.clientY - rect.top;

    const cursorXMap = cursorX / width;
    const cursorYMap = cursorY / height;

    setDestination([
      (cursorY * 100) / img.clientHeight,
      (cursorX * 100) / (img.clientWidth * 2),
    ]);
    return [cursorXMap, cursorYMap];
  }

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      showMap = false;
      trail = [];
      setPath("");
      setUserImage(undefined);
    } else {
      showMap = true;
      // Resend Target
      console.log("Resend target:", coords);
      if (coords) {
        try {
          manager.send("gui", `pick${coords}`);
        } catch {
          /* empty */
        }
      }
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
      <img
        src={CityMap}
        alt=""
        ref={canvasRef}
        className="exercise-canvas"
        id="exercise-img"
        onClick={function pickLoc(event) {
          const data = destinationPicker(event);
          coords = data;
          try {
            manager.send("gui", `pick${data}`);
          } catch {
            /* empty */
          }
        }}
      />
      {carPose && (
        <img
          src={Car}
          id="car"
          style={{
            rotate: "z " + carPose[2] + "rad",
            top: carPose[0] - 5,
            left: carPose[1] - 5,
          }}
        />
      )}
      {destination && (
        <div
          className="target"
          style={{
            top: `${destination[0]}%`,
            left: `calc(${destination[1]}% - ${10}px)`,
          }}
        />
      )}
      {path && (
        <svg
          height="100%"
          width="50%"
          xmlns="http://www.w3.org/2000/svg"
          style={{ zIndex: 2, left: "50%", position: "absolute" }}
        >
          <path
            xmlns="http://www.w3.org/2000/svg"
            d={path}
            style={{
              strokeWidth: "2px",
              strokeLinejoin: "round",
              stroke: "green",
              fill: "none",
            }}
          />
        </svg>
      )}
      <WebGUIImage id="user-img" style={{ left: "50%" }} src={userImage} />
    </WebGUIContainer>
  );
}

export default WebGUI;
