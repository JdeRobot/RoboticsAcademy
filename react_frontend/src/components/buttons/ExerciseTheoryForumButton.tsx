import * as React from "react";
import { ButtonGroup } from "@mui/material";
import IconButton from "@mui/material/IconButton";
import SchoolOutlinedIcon from "@mui/icons-material/SchoolOutlined";
import CodeOutlinedIcon from "@mui/icons-material/CodeOutlined";
import CommentOutlinedIcon from "@mui/icons-material/CommentOutlined";
import RoboticsTheme from "Components/RoboticsTheme.tsx";

import "../../styles/buttons/ExerciseTheoryForumButton.css";

interface ExerciseTheoryForumButtonProps {
  url: string;
}

const ExerciseTheoryForumButton: React.FC<ExerciseTheoryForumButtonProps> = ({ url }) => {
  return (
    <RoboticsTheme>
      <ButtonGroup color={"loading"} variant={"contained"}>
        {url && (
          <IconButton href={url} target="_blank" color="secondary">
            <SchoolOutlinedIcon />
          </IconButton>
        )}
        <IconButton
          href="https://forum.unibotics.org/"
          target="_blank"
          color="secondary"
        >
          <CommentOutlinedIcon />
        </IconButton>
      </ButtonGroup>
    </RoboticsTheme>
  );
};

export default ExerciseTheoryForumButton;
