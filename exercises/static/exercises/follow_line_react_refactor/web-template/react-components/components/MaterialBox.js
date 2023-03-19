import {Box} from "@mui/system";
import RoboticsTheme from "Components/RoboticsTheme";

const MaterialBox = (props) => {
  return (
    <RoboticsTheme>
      <Box id={props.id}>
        {props.children}
      </Box>
    </RoboticsTheme>
  );
};

export default MaterialBox;