import { Button } from "@mui/material";
import VrpanoOutlinedIcon from "@mui/icons-material/VrpanoOutlined";
import * as React from "react";
import PropTypes from "prop-types";

const GazeboButton = (props) => {
  const { changeGzWeb } = React.useContext(props.context);
  return (
    <Button
      id={"gazebo_button"}
      size={"medium"}
      variant="contained"
      color={"secondary"}
      component="span"
      sx={{ m: 1 }}
      title={"Activate the simulation Screen"}
      onClick={changeGzWeb}
      startIcon={<VrpanoOutlinedIcon />}
    >
      View Sim
    </Button>
  );
};
GazeboButton.propTypes = {
  context: PropTypes.any,
};
export default GazeboButton;
