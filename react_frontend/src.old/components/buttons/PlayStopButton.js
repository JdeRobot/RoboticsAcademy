import PlayCircleOutlineOutlinedIcon from "@mui/icons-material/PlayCircleOutlineOutlined";
import StopCircleOutlinedIcon from "@mui/icons-material/StopCircleOutlined";
import { Fab } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";

const PlayStopButton = (props) => {
  const { playState, start, stop } = React.useContext(props.context);
  return (
    <Fab
      id={"submit"}
      color={playState ? "success" : "secondary"}
      sx={{ m: 0.5, position: "fixed", bottom: 2, right: 2 }}
      onClick={playState ? start : stop}
      variant={"outlined"}
    >
      {playState ? (
        <PlayCircleOutlineOutlinedIcon />
      ) : (
        <StopCircleOutlinedIcon />
      )}
    </Fab>
  );
};

PlayStopButton.propTypes = {
  context: PropTypes.any,
};

export default PlayStopButton;
