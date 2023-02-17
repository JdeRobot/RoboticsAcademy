import { Button } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";
import TerminalOutlinedIcon from "@mui/icons-material/TerminalOutlined";

const CameraButton = (props) => {
  const { changeVisualization, visualization } = React.useContext(
    props.context
  );
  return (
    <Button
      id={"console_button"}
      size={"medium"}
      variant="contained"
      color={"secondary"}
      component="span"
      sx={{ m: 1 }}
      title={"Open the console"}
      onClick={() => {
        changeVisualization({
          ...visualization,
          specific: !visualization.specific,
        });
      }}
      startIcon={<TerminalOutlinedIcon />}
    >
      View Camera
    </Button>
  );
};
CameraButton.propTypes = {
  context: PropTypes.any,
};
export default CameraButton;
