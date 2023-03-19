import * as React from "react";
import {ButtonGroup} from "@mui/material";
import IconButton from "@mui/material/IconButton";
import SchoolOutlinedIcon from "@mui/icons-material/SchoolOutlined";
import CodeOutlinedIcon from "@mui/icons-material/CodeOutlined";
import CommentOutlinedIcon from "@mui/icons-material/CommentOutlined";
import RoboticsTheme from "Components/RoboticsTheme";
import {useEffect, useState} from "react";

const ExerciseTheoryForumButton = (props) => {
  const [mode, setMode] = useState("exercise");

  useEffect(() => {
    setView();
  })

  function setView() {
    const views = {
      "exercise": document.getElementById('content_exercise'),
      "theory": document.getElementById('theory-view'),
      "forum": document.getElementById('forum-view')
    };

    const keys = Object.keys(views);

    for(let i=0, length=keys.length; i < length; ++i) {
      let key = keys[i];
      if(views[key]) {
        views[key].style.display = key==mode ? "block" : "none";
      }
    }
  }

  const buttonClick = (event) => {
    const name = event.currentTarget.id;
    console.log(`Clicked on ${name}`);
    setMode(name);
    setView();
  }

  return (
    <RoboticsTheme>
      <ButtonGroup color={"loading"} variant={"contained"}>
        <IconButton component={"span"} id={"exercise"} onClick={buttonClick}>
          <CodeOutlinedIcon/>
        </IconButton>
        <IconButton component={"span"} id={"theory"} onClick={buttonClick}>
          <SchoolOutlinedIcon/>
        </IconButton>
        <IconButton id={"forum"} component={"span"}  onClick={buttonClick}>
          <CommentOutlinedIcon/>
        </IconButton>
      </ButtonGroup>
    </RoboticsTheme>
  );
}

export default ExerciseTheoryForumButton;