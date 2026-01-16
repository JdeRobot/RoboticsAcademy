/* eslint-disable react/no-unknown-property */
import { Points, OrbitControls, Grid, Point } from "@react-three/drei";
import { Canvas, Color, useFrame, useThree, Vector3 } from "@react-three/fiber";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { useEffect, useState } from "react";
import React from "react";

interface WebGUIPoint {
  pose: Vector3;
  color: Color;
}

const WebGUI3D = ({
  id,
  style,
  toPaint,
  reset,
  setReset,
  refreshPoints,
}: {
  id?: string;
  style?: object;
  reset: boolean;
  setReset: (reset: boolean) => void;
  toPaint?: number[][];
  refreshPoints?: boolean;
}) => {
  const [points, addPoints] = useState<WebGUIPoint[]>([]);
  const theme = useAcademyTheme();

  useEffect(() => {
    if (reset) {
      addPoints([]);
      setReset(false);
    }
  }, [reset]);

  useEffect(() => {
    if (toPaint === undefined) {
      return;
    }

    const new_points = [];

    for (let i = 0; i < toPaint.length; i++) {
      const pose: Vector3 = [toPaint[i][0], toPaint[i][1], toPaint[i][2]];
      const color: Color = [
        toPaint[i][3] / 255,
        toPaint[i][4] / 255,
        toPaint[i][5] / 255,
      ];
      new_points.push({ pose, color });
    }

    if (refreshPoints) {
      addPoints(new_points);
    } else {
      addPoints(points.concat(new_points));
    }
  }, [toPaint]);

  return (
    <div id={id} style={style}>
      <Canvas camera={{ position: [20, 50, 100] }}>
        <Scene />
        <axesHelper args={[40]} />
        <ambientLight color={0xffffff} intensity={0.4} />
        <pointLight
          color={0xffffff}
          intensity={1}
          distance={100}
          position={[10, 10, 10]}
        />
        <pointLight
          color={0xffffff}
          intensity={1}
          distance={100}
          position={[20, 20, 20]}
        />
        <pointLight
          color={0xffffff}
          intensity={1}
          distance={100}
          position={[30, 30, 30]}
        />
        <pointLight
          color={0xffffff}
          intensity={1}
          distance={100}
          position={[40, 40, 40]}
        />
        <Grid infiniteGrid sectionColor={theme.viewer3d?.grid} />

        {points.map((point: WebGUIPoint, index) => (
          <Points key={`3d-point-${index}`}>
            <Point position={point.pose} color={point.color} size={1.5} />
            <pointsMaterial vertexColors />
          </Points>
        ))}
        <OrbitControls />
      </Canvas>
    </div>
  );
};

function Scene() {
  const { scene, camera, gl } = useThree();
  const theme = useAcademyTheme();

  useFrame(() => {
    gl.setClearColor(theme.viewer3d.background);
    gl.render(scene, camera);
  }, 1);

  return null;
}

export default WebGUI3D;
