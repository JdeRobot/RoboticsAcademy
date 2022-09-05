import { Button } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";
import VideogameAssetOutlinedIcon from "@mui/icons-material/VideogameAssetOutlined";

const TeleOpButton = (props) => {
  const { teleOpButtonClick } = React.useContext(props.context);
  return (
    <Button
            id={"teleop_button"}
            size={"medium"}
            variant="contained"
            color={"secondary"}
            component="span"
            sx={{ m: 1 }}
            onClick={teleOpButtonClick}
            title={"Use the arrow keys to operate the F1"}
            startIcon={<VideogameAssetOutlinedIcon />}
          >
            Teleoperate
          </Button>
  );
};
TeleOpButton.propTypes = {
  context: PropTypes.any,
};
export default TeleOpButton;
