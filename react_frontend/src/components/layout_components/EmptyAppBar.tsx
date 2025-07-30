import React from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import { Box } from "@mui/material";
import RoboticsTheme from "Components/RoboticsTheme.tsx";
import ExerciseTheoryForumButton from "../buttons/ExerciseTheoryForumButton.tsx";

interface MainAppBarProps {
  url?: string;
  exerciseName?: string;
}

const MainAppBar: React.FC<MainAppBarProps> = ({ url }) => {
  return (
    <RoboticsTheme>
      <AppBar position="relative">
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
            <a href="http://127.0.0.1:7164/exercises/">
              <img
                src="/static/exercises/assets/img/logo.gif"
                style={{ objectFit: "cover" }}
                width={50}
              />
            </a>
          </Box>
          <Box>
            <ExerciseTheoryForumButton url={url} />
          </Box>
        </Toolbar>
      </AppBar>
    </RoboticsTheme>
  );
};

export default MainAppBar;
