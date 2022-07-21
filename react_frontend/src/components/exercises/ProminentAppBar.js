import * as React from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import IconButton from "@mui/material/IconButton";
import Image from "mui-image";
import LoadingButton from "@mui/lab/LoadingButton";
import {
  Box,
  Button,
  ButtonGroup,
  DialogActions,
  DialogContent,
  Typography,
} from "@mui/material";
import ConnectingAirportsIcon from "@mui/icons-material/ConnectingAirports";
import LaunchIcon from "@mui/icons-material/Launch";
import HelpCenterOutlinedIcon from "@mui/icons-material/HelpCenterOutlined";
import SchoolOutlinedIcon from "@mui/icons-material/SchoolOutlined";
import CodeOutlinedIcon from "@mui/icons-material/CodeOutlined";
import CommentOutlinedIcon from "@mui/icons-material/CommentOutlined";
import ExerciseContext from "../../contexts/ExerciseContext";
import ViewContext from "../../contexts/ViewContext";
import RoboticsTheme from "../RoboticsTheme.js";
import BootstrapDialogTitle from "./InfoModalView";
import BootstrapDialog from "./InfoModalView";

function ProminentAppBar() {
  const [open, setOpen] = React.useState(false);

  const {
    startSim,
    connectionState,
    launchState,
    connectionButtonClick,
    launchButtonClick,
    launchLevel,
  } = React.useContext(ExerciseContext);
  const {
    theoryMode,
    codeMode,
    forumMode,
    openTheory,
    openExercise,
    openForum,
  } = React.useContext(ViewContext);
  React.useEffect(() => {
    const onPageLoad = () => {
      startSim(0);
      // connectionButton.current.prop('disabled',true);
    };
    const onUnload = () => {
      startSim(2);
    };
    if (document.readyState === "complete") {
      onPageLoad();
    } else {
      window.addEventListener("load", onPageLoad);
      window.addEventListener("onbeforeunload", onUnload);
      return () => {
        window.removeEventListener("load", onPageLoad);
        window.removeEventListener("onbeforeunload", onUnload);
      };
    }
  }, []);

  const handleClickOpen = () => {
    setOpen(true);
  };
  const handleClose = () => {
    setOpen(false);
  };

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
            <LoadingButton
              id={"connection-button"}
              onClick={connectionButtonClick}
              startIcon={<ConnectingAirportsIcon />}
              variant="contained"
              loadingPosition="start"
              loading={connectionState === "Connecting"}
              color={
                connectionState === "Connecting"
                  ? "loading"
                  : connectionState === "Connect"
                  ? "notConnected"
                  : "success"
              }
              sx={{ marginX: 1 }}
              size={"small"}
              // disabled={connectionState == "Connecting"}
            >
              {connectionState}
            </LoadingButton>
            <LoadingButton
              id={"launch-button"}
              startIcon={<LaunchIcon />}
              onClick={launchButtonClick}
              variant="contained"
              loading={launchState === "Launching"}
              color={
                launchState === "Launching"
                  ? "loading"
                  : launchState === "Launch"
                  ? "notConnected"
                  : "success"
              }
              loadingPosition="start"
              sx={{ marginX: 1 }}
              size={"small"}
            >
              {launchState === "Launching"
                ? `${launchState} ${launchLevel}`
                : launchState}
            </LoadingButton>
            <IconButton
              id={"info-modal"}
              onClick={handleClickOpen}
              sx={{ marginX: 1 }}
            >
              <HelpCenterOutlinedIcon />
            </IconButton>
          </Box>
          <Typography variant="h5">Follow Line exercise</Typography>
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

export default ProminentAppBar;
