import { Button } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";
import PreviewIcon from "@mui/icons-material/Preview";

const CameraButton = (props) => {
  const { changeVisualization, visualization } = React.useContext(
    props.context
  );

  const [disabled, setDisabled] = React.useState(true);

  React.useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "ready") {
        setDisabled(false);
      }
    };

    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
      callback
    );

    return () => {
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
        callback
      );
    };
  }, []);
  return (
    <Button
      id={"console_button"}
      size={"medium"}
      variant="contained"
      disabled={disabled}
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
      startIcon={<PreviewIcon />}
    >
      GUI
    </Button>
  );
};
CameraButton.propTypes = {
  context: PropTypes.any,
};
export default CameraButton;
