import * as React from "react";
import {ButtonGroup} from "@mui/material";
import IconButton from "@mui/material/IconButton";
import SchoolOutlinedIcon from "@mui/icons-material/SchoolOutlined";
import CodeOutlinedIcon from "@mui/icons-material/CodeOutlined";
import CommentOutlinedIcon from "@mui/icons-material/CommentOutlined";
import RoboticsTheme from "Components/RoboticsTheme";
import {useEffect, useState} from "react";

import "../../css/buttons/ExerciseTheoryForumButton.css";

const ExerciseTheoryForumButton = (props) => {
  const [mode, setMode] = useState("exercise");

  useEffect(() => {
    setView();
  })

  function setView() {
    const views = {
      "exercise": document.getElementById('content-exercise'),
      "theory": document.getElementById('theory-view'),
      "forum": document.getElementById('forum-view')
    };

    const keys = Object.keys(views);

    for(let i=0, length=keys.length; i < length; ++i) {
      let key = keys[i];
      if(views[key]) {
        if(key==mode) {
          views[key].classList.remove("view-hidden");
        } else {
          views[key].classList.add("view-hidden");
        }
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
        <IconButton component={"span"} id={"exercise"} onClick={buttonClick}
                    color={mode==="exercise" ? "success" : "secondary"}>
          <CodeOutlinedIcon/>
        </IconButton>
        <IconButton component={"span"} id={"theory"} onClick={buttonClick}
                    color={mode==="theory" ? "success" : "secondary"}>
          <SchoolOutlinedIcon/>
        </IconButton>
        <IconButton id={"forum"} component={"span"}  onClick={buttonClick}
                    color={mode==="forum" ? "success" : "secondary"}>
          <CommentOutlinedIcon/>
        </IconButton>
      </ButtonGroup>
    </RoboticsTheme>
  );
}

export default ExerciseTheoryForumButton;