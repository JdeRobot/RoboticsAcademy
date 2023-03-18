import * as React from "react";
import {ButtonGroup} from "@mui/material";
import IconButton from "@mui/material/IconButton";
import SchoolOutlinedIcon from "@mui/icons-material/SchoolOutlined";
import CodeOutlinedIcon from "@mui/icons-material/CodeOutlined";
import CommentOutlinedIcon from "@mui/icons-material/CommentOutlined";
import RoboticsTheme from "Components/RoboticsTheme";

const ExerciseTheoryForumButton = (props) => {
  return (
    <RoboticsTheme>
      <ButtonGroup color={"loading"} variant={"contained"}>
        <IconButton component={"span"} id={"open-exercise"}>
          <CodeOutlinedIcon/>
        </IconButton>
        <IconButton component={"span"} id={"open-theory"}>
          <SchoolOutlinedIcon/>
        </IconButton>
        <IconButton id={"open-forum"} component={"span"}>
          <CommentOutlinedIcon/>
        </IconButton>
      </ButtonGroup>
    </RoboticsTheme>
  );
}

export default ExerciseTheoryForumButton;