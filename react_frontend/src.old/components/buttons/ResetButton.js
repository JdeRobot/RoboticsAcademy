import { Button } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";
import RestartAltOutlinedIcon from "@mui/icons-material/RestartAltOutlined";

const ResetButton = (props) => {
  const { resetSim } = React.useContext(props.context);
  return (
    <Button
      id={"reset"}
      color={"secondary"}
      startIcon={<RestartAltOutlinedIcon />}
      sx={{ m: 0.5 }}
      onClick={resetSim}
      variant={"outlined"}
    >
      Reset
    </Button>
  );
};

ResetButton.propTypes = {
  context: PropTypes.any,
};

export default ResetButton;
