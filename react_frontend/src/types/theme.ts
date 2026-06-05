import { Theme } from "jderobot-ide-interface";

export interface AcademyTheme extends Theme {
  switch: (themeType: string) => void;
  viewer3d: {
    grid: string;
    background: string;
  };
}
