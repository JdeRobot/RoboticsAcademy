import * as React from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import IconButton from "@mui/material/IconButton";
import Image from "mui-image";
import LoadingButton from "@mui/lab/LoadingButton";
import { Box, Button, ButtonGroup, Typography } from "@mui/material";
import ConnectingAirportsIcon from "@mui/icons-material/ConnectingAirports";
import LaunchIcon from "@mui/icons-material/Launch";
import HelpCenterOutlinedIcon from "@mui/icons-material/HelpCenterOutlined";
import SchoolOutlinedIcon from "@mui/icons-material/SchoolOutlined";
import CodeOutlinedIcon from "@mui/icons-material/CodeOutlined";
import CommentOutlinedIcon from "@mui/icons-material/CommentOutlined";
import ExerciseContext from "../../contexts/ExerciseContext";
import ViewContext from "../../contexts/ViewContext";
import RoboticsTheme from "../RoboticsTheme.js";
function ProminentAppBar() {
  const { startSim } = React.useContext(ExerciseContext);
  const {
    theoryMode,
    codeMode,
    forumMode,
    openTheory,
    openExercise,
    openForum,
  } = React.useContext(ViewContext);
  const connectionButton = React.useRef(null);
  const infoModal = React.useRef(null);
  React.useEffect(() => {
    const onPageLoad = () => {
      startSim(0);
      // connectionButton.current.prop('disabled',true);
    };
    const onUnload = () => {
      startSim(2);
    };
    if (document.readyState == "complete") {
      onPageLoad();
    } else {
      window.addEventListener("load", onPageLoad);

      return () => window.removeEventListener("load", onPageLoad);
    }
  }, []);
  const connectionButtonClick = () => {};
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
            <Image
              src="/static/common/img/logo.gif"
              fit={"cover"}
              width={50}
            ></Image>
            <LoadingButton
              id={"connection-button"}
              ref={connectionButton}
              onClick={connectionButtonClick}
              startIcon={<ConnectingAirportsIcon />}
              variant="contained"
              loadingIndicator="Connecting…"
              color={"success"}
              sx={{ marginX: 1 }}
              size={"small"}
              disabled={true}
            >
              Connect
            </LoadingButton>
            <Button
              id={"launch-button"}
              startIcon={<LaunchIcon />}
              variant="contained"
              color={"secondary"}
              sx={{ marginX: 1 }}
              size={"small"}
            >
              Launch
            </Button>
            <Button
              ref={infoModal}
              id={"info-modal"}
              sx={{ marginX: 1 }}
              startIcon={<HelpCenterOutlinedIcon />}
            />
          </Box>
          <Typography variant="h5">Follow Line exercise</Typography>
          <ButtonGroup color={"secondary"} variant={"contained"}>
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

export default ProminentAppBar;
