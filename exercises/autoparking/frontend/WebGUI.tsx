import React, { useState, useEffect, useRef } from "react";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUI3D from "Components/exercise/WebGUI3D";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";

const WebGUI = () => {
  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);
  const [reset, setReset] = useState(false);
  const [pointsToPaint, setPointsToPaint] = useState<number[][] | undefined>();
  const lidarFramesRef = useRef<number[][][]>([]);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const updateCallback = (updateData: unknown) => {
    let data = updateData as any;
    data = data.update;

    if (typeof data.lidar === "string") {
      try {
        const points = JSON.parse(data.lidar) as number[][];
        if (Array.isArray(points) && points.length > 0) {
          lidarFramesRef.current.push(points);
          if (lidarFramesRef.current.length > 3) {
            lidarFramesRef.current.shift();
          }
          const mergedPoints = lidarFramesRef.current.reduce<number[][]>(
            (acc, frame) => acc.concat(frame),
            []
          );
          setPointsToPaint(mergedPoints);
        } else {
          // If LiDAR frame is empty, clear buffered frames to avoid stale ghosts.
          lidarFramesRef.current = [];
          setPointsToPaint([]);
        }
      } catch {
        // Ignore malformed websocket frames and keep last valid render.
      }
    }
    if (data.map) {
      lidarFramesRef.current = [];
      const map_data = JSON.parse(data.map);
      var pdata: number[][] = [];

      // Pose: 1.000000 -2.60000 0.500000
      map_data.lasers[0].forEach((element: number[]) => {
        if (element[0] < 600) {
          const x = Math.cos(element[1]) * (element[0] / 2.5);
          const y = Math.sin(element[1]) * (element[0] / 2.5);

          pdata.push([x, 0.5, -y - 20, 255, 0, 0]);
        }
      });

      // Pose: -1.100000 0.000000 0.500000
      map_data.lasers[1].forEach((element: number[]) => {
        if (element[0] < 600) {
          const x = Math.cos(element[1]) * (element[0] / 2.5);
          const y = Math.sin(element[1]) * (element[0] / 2.5);

          pdata.push([y + 9, 0.5, x, 0, 255, 0]);
        }
      });

      // Pose: 1.000000 2.60000 0.500000
      map_data.lasers[2].forEach((element: number[]) => {
        if (element[0] < 600) {
          const x = Math.cos(element[1]) * (element[0] / 2.5);
          const y = Math.sin(element[1]) * (element[0] / 2.5);

          pdata.push([-x, 0.5, y + 20, 0, 0, 255]);
        }
      });
      setPointsToPaint(pdata);
    }
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setReset(true);
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  return (
    <WebGUIContainer>
      <WebGUI3D
        toPaint={pointsToPaint}
        reset={reset}
        setReset={setReset}
        refreshPoints
        id="canvas"
        style={{ height: "100%", width: "100%" }}
      />
    </WebGUIContainer>
  );
};

export default WebGUI;
