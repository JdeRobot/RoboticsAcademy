import { Box } from "@mui/system";
import RoboticsTheme from "Components/RoboticsTheme.tsx";
import React, { ReactNode } from "react";

interface MaterialBoxProps {
  id?: string;
  children: ReactNode;
}

const MaterialBox: React.FC<MaterialBoxProps> = ({ id, children }) => {
  return (
    <RoboticsTheme>
      <Box id={id}>{children}</Box>
    </RoboticsTheme>
  );
};

export default MaterialBox;
