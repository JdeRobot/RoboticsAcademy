import { Button } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";
import TerminalOutlinedIcon from "@mui/icons-material/TerminalOutlined";

const ConsoleButton = (props) => {
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
      disabled={disabled}
      variant="contained"
      color={"secondary"}
      component="span"
      sx={{ m: 1 }}
      title={"Open the console"}
      onClick={() => {
        changeVisualization({
          ...visualization,
          console: !visualization.console,
        });
      }}
      startIcon={<TerminalOutlinedIcon />}
    >
      Console
    </Button>
  );
};
ConsoleButton.propTypes = {
  context: PropTypes.any,
};
export default ConsoleButton;
