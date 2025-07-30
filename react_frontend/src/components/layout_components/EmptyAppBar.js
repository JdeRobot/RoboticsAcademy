import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import { Box } from "@mui/material";
import RoboticsTheme from "Components/RoboticsTheme.js";
import PropTypes from "prop-types";
import ExerciseTheoryForumButton from "../buttons/ExerciseTheoryForumButton";

function MainAppBar(props) {
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
                fit={"cover"}
                width={50}
              />
            </a>
          </Box>
          <Box>
            <ExerciseTheoryForumButton
              url={props.url}
            ></ExerciseTheoryForumButton>
          </Box>
        </Toolbar>
      </AppBar>
    </RoboticsTheme>
  );
}

MainAppBar.propTypes = {
  exerciseName: PropTypes.string,
};

export default MainAppBar;
