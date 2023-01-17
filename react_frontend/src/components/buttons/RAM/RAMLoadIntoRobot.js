import SmartToyOutlinedIcon from "@mui/icons-material/SmartToyOutlined";
import { Button } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";

const RAMLoadIntoRobot = (props) => {
  const { submitCode } = React.useContext(props.context);

  return (
    <Button
      id={"loadIntoRobot"}
      color={"secondary"}
      onClick={submitCode}
      startIcon={<SmartToyOutlinedIcon />}
      sx={{ m: 0.5 }}
      variant={"outlined"}
    >
      Load in robot
    </Button>
  );
};
RAMLoadIntoRobot.propTypes = {
  context: PropTypes.any,
};

export default RAMLoadIntoRobot;
