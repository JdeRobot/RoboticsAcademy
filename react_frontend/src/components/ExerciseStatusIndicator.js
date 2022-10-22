import { keyframes } from "@mui/system";
import React from "react";
import PropTypes from "prop-types";
import { Box, Typography } from "@mui/material";
import { styled } from "@mui/material/styles";

const blink = keyframes`
  from { opacity: 0; }
  to { opacity: 1; }
`;

// eslint-disable-next-line no-unused-vars
const BlinkingIndicator = styled("div")(({ status }) => ({
  backgroundColor:
    status === "ACTIVE" ? "green" : status === "INACTIVE" ? "red" : "orange",
  width: 10,
  height: 10,
  borderRadius: "50%",
  borderColor: "black",
  animation: `${blink} 3s linear infinite`,
}));

const Indicator = styled("div")(({ status }) => ({
  backgroundColor:
    status === "ACTIVE" ? "green" : status === "INACTIVE" ? "red" : "orange",
  width: 10,
  height: 10,
  borderRadius: "50%",
  borderColor: "black",
  // animation: `${blink} 3s linear infinite`,
}));

const ExerciseStatusIndicator = (props) => {
  return (
    <Box
      sx={{
        display: "inline-flex",
        flexDirection: "row",
        alignItems: "center",
        justifyContent: "center",
        border: 1,
        borderColor: "white",
        py: 0.5,
        pl: 1,
        pr: 2,
        borderRadius: "16px",
      }}
    >
      <Typography
        color={"white"}
        variant="h8"
        component="div"
        sx={{ marginRight: 2 }}
      >
        Status
      </Typography>
      <Indicator status={props.status} />
    </Box>
  );
};

ExerciseStatusIndicator.propTypes = {
  status: PropTypes.string,
};

export default ExerciseStatusIndicator;
