import {Box} from "@mui/system";

const MaterialBox = (props) => {
    return (
        <Box>
            {props.children}
        </Box>
    );
};

export default MaterialBox;