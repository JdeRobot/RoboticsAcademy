import * as React from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import IconButton from "@mui/material/IconButton";
import Image from "mui-image";
import { Box, ButtonGroup, Typography } from "@mui/material";
import SchoolOutlinedIcon from "@mui/icons-material/SchoolOutlined";
import CodeOutlinedIcon from "@mui/icons-material/CodeOutlined";
import CommentOutlinedIcon from "@mui/icons-material/CommentOutlined";
import ViewContext from "../../contexts/ViewContext";
import RoboticsTheme from "../RoboticsTheme.js";
import PropTypes from "prop-types";
import { ConnectionIndicator } from "./RAM/ConnectionIndicator";
import { LaunchIndicator } from "./RAM/LaunchIndicator";
import { useLoad } from "../../hooks/useLoad";
import { useUnload } from "../../hooks/useUnload";

function MainAppBar(props) {
  const {
    theoryMode,
    codeMode,
    forumMode,
    openTheory,
    openExercise,
    openForum,
  } = React.useContext(ViewContext);
  const config = JSON.parse(
    document.getElementById("exercise-config").textContent
  );
  config.height = window.innerHeight / 2;
  config.width = window.innerWidth / 2;

  useLoad(() => {
    window.RoboticsExerciseComponents.commsManager.connect().then(() => {
      window.RoboticsExerciseComponents.commsManager.launch(config);
    });
  });

  useUnload(() => {
    window.RoboticsExerciseComponents.commsManager.terminate();
  });
  return (
    <RoboticsTheme>
      <AppBar position="static">
        <Toolbar
          sx={{
            display: "flex",
            flexWrap: "wrap",
            justifyContent: "space-between",
          }}
        >
          <Box
            sx={{
              display: "inline-flex",
              justifyContent: "space-between",
              alignItems: "center",
            }}
          >
            <Image src="/static/common/img/logo.gif" fit={"cover"} width={50} />
            <Box sx={{ display: "flex", gap: "10px", marginLeft: "10px" }}>
              <ConnectionIndicator></ConnectionIndicator>
              <LaunchIndicator></LaunchIndicator>
            </Box>
          </Box>
          <Typography variant="h5">{props.exerciseName}</Typography>
          <ButtonGroup color={"loading"} variant={"contained"}>
            <IconButton
              component={"span"}
              id={"open-theory"}
              onClick={openTheory}
              color={theoryMode ? "success" : "secondary"}
            >
              <SchoolOutlinedIcon />
            </IconButton>
            <IconButton
              component={"span"}
              id={"open-exercise"}
              onClick={openExercise}
              color={codeMode ? "success" : "secondary"}
            >
              <CodeOutlinedIcon />
            </IconButton>
            <IconButton
              id={"open-forum"}
              onClick={openForum}
              component={"span"}
              color={forumMode ? "success" : "secondary"}
            >
              <CommentOutlinedIcon />
            </IconButton>
          </ButtonGroup>
        </Toolbar>
      </AppBar>
    </RoboticsTheme>
  );
}

MainAppBar.propTypes = {
  context: PropTypes.any,
  exerciseName: PropTypes.string,
};

export default MainAppBar;
