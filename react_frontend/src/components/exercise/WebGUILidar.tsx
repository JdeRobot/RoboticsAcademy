/* eslint-disable react/react-in-jsx-scope */
/* eslint-disable react/no-unknown-property */
import { Points, ThreeDimensionalCanvas, ThreeDimensionalControls, type PointBaseProps } from '@arizeai/point-cloud';
import { Grid } from "@react-three/drei";
import { Color } from "@react-three/fiber";
import { useAcademyTheme } from "Contexts/AcademyThemeContext";
import { useEffect, useState } from "react";

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
  const [points, addPoints] = useState<PointBaseProps[]>([]);
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

    const new_points: PointBaseProps[] = [];

    for (let i = 0; i < toPaint.length; i++) {
      const color: Color = [
        toPaint[i][3],
        toPaint[i][4],
        toPaint[i][5],
      ];
      new_points.push({ position: [toPaint[i][0], toPaint[i][1], toPaint[i][2]], metaData: {color:color} });
    }

    if (refreshPoints) {
      addPoints(new_points);
    } else {
      addPoints(points.concat(new_points));
    }
  }, [toPaint]);

  const colorByFn = (data: PointBaseProps) => {
    const { color } = data.metaData;
    return `rgb(${color[0]},${color[1]},${color[2]})`;
  };

  return (
    <div id={id} style={style}>
      <ThreeDimensionalCanvas camera={{ position: [20, 50, 100] }}>
        <axesHelper args={[40]} />
        <ambientLight color={0xffffff} intensity={0.75} />
        <Grid infiniteGrid sectionColor={theme.viewer3d?.grid} />
        <ThreeDimensionalControls />
        <Points data={points} pointProps={{ color: colorByFn,radius:0.75 }} opacity={0.75} pointShape="sphere"/>
      </ThreeDimensionalCanvas>
    </div>
  );
};

export default WebGUI3D;
